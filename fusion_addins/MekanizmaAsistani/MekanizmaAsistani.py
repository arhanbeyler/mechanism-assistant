"""
Mekanizma Asistani — Fusion 360 eklentisi.

1) Panel: Fusion'in altinda sohbet kutusu. Kullanici cumleyi yazar; LLM + kinematik
   motor ARKA PLANDA calisir (Fusion donmaz), sonuc ana is parcaciginda cizilir.
2) Kopru: 127.0.0.1 uzerinde, token korumali kucuk HTTP sunucusu. Gelistirme
   sirasinda Claude'un Fusion'da kod calistirip ekran goruntusu almasini saglar.

Kural: Fusion API'si sadece ANA is parcacigindan cagrilabilir. Arka plan is
parcaciklari isi run_on_main() ile ana is parcacigina (custom event) devreder.
"""

import contextlib
import importlib
import io
import json
import os
import secrets
import sys
import threading
import time
import traceback
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import adsk.core
import adsk.fusion

ADDIN_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(os.path.dirname(ADDIN_DIR))  # proje klasoru
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

PALETTE_ID = "MekanizmaAsistaniPalette"
CMD_ID = "MekanizmaAsistaniCmd"
EVENT_ID = "MekanizmaAsistaniMainThread"
BRIDGE_PORT = 8765
TOKEN_FILE = os.path.join(ROOT, ".bridge_token")
LOG_FILE = os.path.join(ROOT, "output", "addin.log")

app = None
ui = None
handlers = []           # Fusion olay isleyicileri canli kalsin diye referans tutulur
pending = {}            # is id -> ana is parcaciginda calisacak fonksiyon
pending_lock = threading.Lock()
server = None


# ---------------------------------------------------------------- yardimcilar

def log(msg):
    try:
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
        with open(LOG_FILE, "a", encoding="utf-8") as fh:
            fh.write(time.strftime("%H:%M:%S ") + msg + "\n")
    except Exception:
        pass


def run_on_main(fn):
    """Herhangi bir is parcacigindan: fn'i Fusion'in ana is parcaciginda calistir."""
    job = uuid.uuid4().hex
    with pending_lock:
        pending[job] = fn
    app.fireCustomEvent(EVENT_ID, job)


def call_on_main(fn, timeout=120):
    """run_on_main + sonucu bekle. Dondurur: {'value': ...} ya da {'error': iz}."""
    done, box = threading.Event(), {}

    def wrapper():
        try:
            box["value"] = fn()
        except Exception:
            box["error"] = traceback.format_exc()
        finally:
            done.set()

    run_on_main(wrapper)
    if not done.wait(timeout):
        return {"error": f"Zaman asimi ({timeout} s): Fusion mesgul olabilir."}
    return box


def reload_modules():
    """Proje modullerini yeniden yukle: kod degisince Fusion'u yeniden baslatmaya gerek yok."""
    import kinematics
    import llm_extract
    import pipeline
    import fusion_lib.fourbar
    for mod in (kinematics, llm_extract, pipeline, fusion_lib.fourbar):
        importlib.reload(mod)
    return pipeline, fusion_lib.fourbar


def send_to_palette(action, payload):
    """Sadece ana is parcacigindan cagrilir."""
    pal = ui.palettes.itemById(PALETTE_ID)
    if pal:
        pal.sendInfoToHTML(action, json.dumps(payload, ensure_ascii=False))


# ---------------------------------------------------------------- tasarim akisi

def start_design(text):
    pipeline, fourbar = reload_modules()

    def status(s):
        run_on_main(lambda: send_to_palette("status", {"text": s}))

    def worker():
        try:
            result = pipeline.design_from_text(text, on_status=status)
        except urllib.error.URLError:
            result = {"status": "error",
                      "message": "Yerel LLM'e (Ollama) ulaşılamadı. Ollama çalışıyor mu?"}
        except Exception as e:
            log("design hata:\n" + traceback.format_exc())
            result = {"status": "error", "message": f"Beklenmeyen hata: {e}"}
        if result.get("status") == "ok":
            status("Fusion'da çiziliyor...")
        run_on_main(lambda: finish_design(result, fourbar))

    threading.Thread(target=worker, daemon=True).start()


def finish_design(result, fourbar):
    if result.get("status") == "ok":
        try:
            fourbar.build(result["fusion"])
            result["drawn"] = True
        except Exception as e:
            log("cizim hata:\n" + traceback.format_exc())
            result["drawn"] = False
            result["message"] += f" Ama çizimde hata oluştu: {e}"
    send_to_palette("result", result)


# ---------------------------------------------------------------- Fusion olaylari

class MainThreadHandler(adsk.core.CustomEventHandler):
    def notify(self, args):
        with pending_lock:
            fn = pending.pop(args.additionalInfo, None)
        if fn:
            try:
                fn()
            except Exception:
                log("ana is parcacigi hata:\n" + traceback.format_exc())


class PaletteHTMLHandler(adsk.core.HTMLEventHandler):
    def notify(self, args):
        try:
            html_args = adsk.core.HTMLEventArgs.cast(args)
            if html_args.action == "design":
                text = json.loads(html_args.data).get("text", "").strip()
                if text:
                    start_design(text)
                html_args.returnData = "started"
            elif html_args.action == "ready":
                html_args.returnData = "ok"
        except Exception:
            log("panel hata:\n" + traceback.format_exc())


class StartupCompletedHandler(adsk.core.ApplicationEventHandler):
    """Fusion acilisini bitirince paneli ac (erken acilan panel arayuz yuklenirken kayboluyor)."""
    def notify(self, args):
        try:
            show_palette()
        except Exception:
            log("acilis paneli hata:\n" + traceback.format_exc())


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        on_execute = CommandExecuteHandler()
        args.command.execute.add(on_execute)
        handlers.append(on_execute)


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        show_palette()


def show_palette():
    pal = ui.palettes.itemById(PALETTE_ID)
    if not pal:
        url = os.path.join(ADDIN_DIR, "palette.html").replace("\\", "/")
        pal = ui.palettes.add(PALETTE_ID, "Mekanizma Asistanı", url, False, True, True, 1000, 300)
        on_html = PaletteHTMLHandler()
        pal.incomingFromHTML.add(on_html)
        handlers.append(on_html)
        pal.dockingState = adsk.core.PaletteDockingStates.PaletteDockStateBottom
    pal.isVisible = True


# ---------------------------------------------------------------- kopru (gelistirme)

TOKEN = None


def bridge_exec(code):
    """Ana is parcaciginda Python kodu calistir, print ciktisini dondur."""
    def fn():
        buf = io.StringIO()
        ns = {"app": app, "ui": ui, "adsk": adsk, "ROOT": ROOT, "reload_modules": reload_modules}
        with contextlib.redirect_stdout(buf):
            exec(code, ns)
        return buf.getvalue()
    return call_on_main(fn)


def bridge_screenshot(width=1400, height=900):
    path = os.path.join(ROOT, "output", "screenshot.png")

    def fn():
        os.makedirs(os.path.dirname(path), exist_ok=True)
        ok = app.activeViewport.saveAsImageFile(path, width, height)
        return path if ok else None
    return call_on_main(fn)


class BridgeHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if not secrets.compare_digest(self.headers.get("X-Token", ""), TOKEN or ""):
            self._reply(403, {"error": "gecersiz token"})
            return
        body = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode("utf-8")
        if self.path == "/exec":
            self._reply(200, bridge_exec(body))
        elif self.path == "/screenshot":
            self._reply(200, bridge_screenshot())
        else:
            self._reply(404, {"error": "bilinmeyen yol"})

    def _reply(self, code, obj):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


class BridgeServer(ThreadingHTTPServer):
    # Windows'ta SO_REUSEADDR ayni portu iki sunucuya actirir (eklenti iki kez
    # baslatilirsa istekler eski sunucuya gider). Kapali: port doluysa hata verir.
    allow_reuse_address = False
    daemon_threads = True


def start_bridge():
    global server, TOKEN
    # Anahtar kalici: eklenti yeniden baslasa da ayni kalir
    try:
        with open(TOKEN_FILE, encoding="utf-8") as fh:
            TOKEN = fh.read().strip() or None
    except OSError:
        TOKEN = None
    if not TOKEN:
        TOKEN = secrets.token_hex(16)
        with open(TOKEN_FILE, "w", encoding="utf-8") as fh:
            fh.write(TOKEN)
    try:
        server = BridgeServer(("127.0.0.1", BRIDGE_PORT), BridgeHandler)
    except OSError as e:
        log(f"kopru baslatilamadi (port {BRIDGE_PORT}): {e}")
        return
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log(f"kopru hazir: 127.0.0.1:{BRIDGE_PORT}")


def warm_up_llm():
    """Modeli onceden VRAM'e yukle: ilk istek 100 s yerine birkac saniye sursun."""
    def worker():
        try:
            import llm_extract
            req = urllib.request.Request(
                "http://localhost:11434/api/generate",
                data=json.dumps({"model": llm_extract.MODEL, "keep_alive": "30m"}).encode(),
                headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=300).read()
            log("LLM isitildi")
        except Exception as e:
            log(f"LLM isitilamadi: {e}")
    threading.Thread(target=worker, daemon=True).start()


# ---------------------------------------------------------------- yasam dongusu

RUNNING_FLAG = "_mekanizma_asistani_running"   # tum eklentilerin paylastigi sys modulunde
owner = False                                   # bu kopya mi baslatti?


def run(context):
    global app, ui, owner
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        # Ayni eklenti iki yoldan kayitli olabilir (kisayol + gercek klasor): tek kopya calissin
        if getattr(sys, RUNNING_FLAG, False):
            log("---- ikinci kopya atlandi (eklenti zaten calisiyor): " + ADDIN_DIR)
            return
        setattr(sys, RUNNING_FLAG, True)
        owner = True
        log("---- eklenti basladi: " + os.path.realpath(__file__))

        custom_event = app.registerCustomEvent(EVENT_ID)
        on_main = MainThreadHandler()
        custom_event.add(on_main)
        handlers.append(on_main)

        cmd_def = ui.commandDefinitions.itemById(CMD_ID) or ui.commandDefinitions.addButtonDefinition(
            CMD_ID, "Mekanizma Asistanı", "Doğal dille mekanizma tasarla (yerel yapay zekâ)")
        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        handlers.append(on_created)
        panel = ui.allToolbarPanels.itemById("SolidScriptsAddinsPanel")
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)

        start_bridge()
        warm_up_llm()
        if app.isStartupComplete:
            show_palette()            # elle Run'a basildi: hemen ac
        else:
            on_startup = StartupCompletedHandler()   # Fusion aciliyor: bitmesini bekle
            app.startupCompleted.add(on_startup)
            handlers.append(on_startup)
    except Exception:
        log("run hata:\n" + traceback.format_exc())
        if ui:
            ui.messageBox("Mekanizma Asistanı başlatılamadı:\n" + traceback.format_exc())


def stop(context):
    global server, owner
    if not owner:
        return          # calisan kopya bu degil: onun kaynaklarina dokunma
    owner = False
    setattr(sys, RUNNING_FLAG, False)
    try:
        if server:
            server.shutdown()
            server.server_close()
            server = None
        pal = ui.palettes.itemById(PALETTE_ID)
        if pal:
            pal.deleteMe()
        panel = ui.allToolbarPanels.itemById("SolidScriptsAddinsPanel")
        ctrl = panel.controls.itemById(CMD_ID) if panel else None
        if ctrl:
            ctrl.deleteMe()
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        app.unregisterCustomEvent(EVENT_ID)
        handlers.clear()
        log("---- eklenti durdu")
    except Exception:
        log("stop hata:\n" + traceback.format_exc())
