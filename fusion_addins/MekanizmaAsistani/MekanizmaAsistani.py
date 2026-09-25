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
import math
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
    import pipeline
    import fusion_lib.fourbar
    root = os.path.normcase(ROOT)
    ours = [m for m in list(sys.modules.values())
            if getattr(m, "__file__", None)
            and os.path.normcase(os.path.realpath(m.__file__)).startswith(root)
            and os.path.normcase(ADDIN_DIR) not in os.path.normcase(os.path.realpath(m.__file__))]
    # bagimliliklar once: pipeline en son (digerlerini import ediyor)
    for mod in sorted(ours, key=lambda m: m.__name__ == "pipeline"):
        importlib.reload(mod)
    return sys.modules["pipeline"], sys.modules["fusion_lib.fourbar"]


def send_to_palette(action, payload):
    """Sadece ana is parcacigindan cagrilir."""
    pal = ui.palettes.itemById(PALETTE_ID)
    if pal:
        pal.sendInfoToHTML(action, json.dumps(payload, ensure_ascii=False))


# ---------------------------------------------------------------- tasarim akisi

def start_preview(params, req):
    """Kaydirici onizlemesi: LLM yok, Fusion'a cizim yok; sadece hesap + panel."""
    pipeline = sys.modules.get("pipeline") or reload_modules()[0]

    def worker():
        try:
            result = pipeline.design_from_params(**params)
        except Exception as e:
            log("onizleme hata:\n" + traceback.format_exc())
            result = {"status": "error", "message": f"Hesap hatası: {e}"}
        result["req"] = req
        run_on_main(lambda: send_to_palette("preview", result))

    threading.Thread(target=worker, daemon=True).start()


def start_band(Q, req):
    pipeline = sys.modules.get("pipeline") or reload_modules()[0]

    def worker():
        try:
            band = pipeline.swing_band(Q)
        except Exception:
            log("band hata:\n" + traceback.format_exc())
            return
        run_on_main(lambda: send_to_palette("band", {"req": req, "Q": Q, "band": band}))

    threading.Thread(target=worker, daemon=True).start()


def start_fea(params):
    """Yerel FEA (gmsh + scikit-fem) ile kol ve krank icin el hesabini dogrula.
    FEA, Fusion'in Python'unda degil bu bilgisayarin normal Python'unda calisir (.local_python)."""
    import subprocess
    pipeline = sys.modules.get("pipeline") or reload_modules()[0]

    def status(s):
        run_on_main(lambda: send_to_palette("status", {"text": s}))

    def worker():
        try:
            r = pipeline.design_from_params(**params)
            st = r.get("strength")
            if r.get("status") != "ok" or not st:
                out = {"ok": False, "message": "FEA için önce kontrollerden geçen ve yükü verilmiş bir tasarım gerekli "
                                              "(Uçtaki yük kaydırıcısını 0'dan büyük yap)."}
            else:
                import strength
                mat = strength.MATERIALS[params.get("material") or "PLA"]
                req = {"lengths": r["lengths"], "section": r["section_mm"], "load": st["load_N"],
                       "material": {"E": mat["E"], "nu": mat["nu"]},
                       "crank_torque": st["input_torque_max_Nmm"]}
                with open(os.path.join(ROOT, ".local_python"), encoding="utf-8") as fh:
                    py = fh.read().strip()
                status("Yerel FEA çalışıyor (ağ oluşturma + çözüm)...")
                proc = subprocess.run([py, os.path.join(ROOT, "fea_local.py"), "--json"],
                                      input=json.dumps(req), capture_output=True, text=True,
                                      timeout=300, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                if proc.returncode != 0:
                    log("FEA hata:\n" + proc.stderr[-3000:])
                    out = {"ok": False, "message": "FEA çalışmadı (ayrıntı: output/addin.log)."}
                else:
                    out = {"ok": True, "links": json.loads(proc.stdout), "material": mat["name"]}
        except Exception as e:
            log("FEA hata:\n" + traceback.format_exc())
            out = {"ok": False, "message": f"FEA hatası: {e}"}
        run_on_main(lambda: send_to_palette("fea", out))

    threading.Thread(target=worker, daemon=True).start()


def start_design(text=None, params=None):
    """Cumleden (LLM ile) ya da kaydirici degerlerinden tasarla ve Fusion'da ciz."""
    pipeline, fourbar = reload_modules()

    def status(s):
        run_on_main(lambda: send_to_palette("status", {"text": s}))

    def worker():
        try:
            if params is not None:
                result = pipeline.design_from_params(**params)
            else:
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


# ---------------------------------------------------------------- dondurme (animasyon)

anim = {"running": False}
STEP_DEG = 4          # her adimda krank acisi
STEP_PAUSE = 0.02     # adimlar arasi bekleme (s)


def crank_motion():
    """Aktif belgedeki mekanizmanin krank mafsali (yoksa None)."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        return None
    for j in design.rootComponent.asBuiltJoints:
        if j.name.startswith("Sabit-Krank"):
            return adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)
    return None


def rotate(mode):
    """mode: 'turn' (bir tam tur) ya da 'toggle' (surekli donmeyi ac/kapat)."""
    if anim["running"]:
        if mode == "toggle":
            anim["running"] = False       # calisan is parcacigi durur ve paneli gunceller
        return
    if crank_motion() is None:
        send_to_palette("status", {"text": "Açık belgede dönecek bir mekanizma yok. Önce 'Fusion'da çiz'."})
        return
    anim["running"] = True
    send_to_palette("anim", {"running": True, "mode": mode})

    # Sinirli acili mekanizma (B9): mafsalin sinirlari varsa tam tur yerine sinirlar arasinda salinim
    motion0 = crank_motion()
    lim = motion0.rotationLimits
    if lim.isMinimumValueEnabled and lim.isMaximumValueEnabled:
        lo, hi, start = lim.minimumValue, lim.maximumValue, motion0.rotationValue
        # "1 tur" = baslangic -> ust sinir -> alt sinir -> baslangic; surekli = ust <-> alt
        targets = [hi, lo, start] if mode == "turn" else None
    else:
        lo = hi = None
        targets = None

    def worker():
        done, goal_i, going_up = 0, 0, True
        step_rad = math.radians(STEP_DEG)
        while anim["running"]:
            if lo is None:                        # tam tur donebilen mekanizma
                if mode == "turn" and done >= 360:
                    break
                delta = step_rad
                done += STEP_DEG
            else:
                box = {}
                ev = threading.Event()
                run_on_main(lambda: (box.__setitem__("v", crank_motion().rotationValue), ev.set()))
                ev.wait(2.0)
                cur = box.get("v", 0.0)
                goal = targets[goal_i] if targets else (hi if going_up else lo)
                if abs(goal - cur) <= step_rad:
                    delta = goal - cur
                    if targets:
                        goal_i += 1
                        if goal_i >= len(targets):
                            anim["running"] = False
                    else:
                        going_up = not going_up
                else:
                    delta = step_rad if goal > cur else -step_rad
            stepped = threading.Event()

            def step(delta=delta):
                try:
                    motion = crank_motion()
                    motion.rotationValue = motion.rotationValue + delta
                except Exception:
                    anim["running"] = False       # belge kapandi vb.
                finally:
                    stepped.set()

            run_on_main(step)
            stepped.wait(2.0)
            time.sleep(STEP_PAUSE)
        anim["running"] = False
        run_on_main(lambda: send_to_palette("anim", {"running": False}))

    threading.Thread(target=worker, daemon=True).start()


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
            try:
                data = json.loads(html_args.data) if html_args.data else {}
            except ValueError:
                data = {}
            if html_args.action == "design":
                text = data.get("text", "").strip()
                if text:
                    start_design(text=text)
                html_args.returnData = "started"
            elif html_args.action == "preview":
                start_preview(data["params"], data.get("req", 0))
                html_args.returnData = "started"
            elif html_args.action == "band":
                start_band(float(data.get("Q", 1.0)), data.get("req", 0))
                html_args.returnData = "started"
            elif html_args.action == "draw":
                anim["running"] = False           # cizmeden once donmeyi durdur
                start_design(params=data["params"])
                html_args.returnData = "started"
            elif html_args.action == "fea":
                start_fea(data["params"])
                html_args.returnData = "started"
            elif html_args.action == "rotate":
                rotate(data.get("mode", "turn"))
                html_args.returnData = "ok"
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
    anim["running"] = False
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
