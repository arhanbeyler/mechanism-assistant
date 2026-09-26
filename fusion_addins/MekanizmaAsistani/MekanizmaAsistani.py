"""
Mechanism Assistant: Fusion 360 add-in.

1) Panel: a chat box + sliders + result card. The user types a sentence; the LLM and the engines run in
   the BACKGROUND (Fusion does not freeze), the result is drawn on the main thread.
2) Bridge: a small token-protected HTTP server on 127.0.0.1. During development it lets Claude run code
   in Fusion and take screenshots.

Rule: the Fusion API may only be called from the MAIN thread. Background threads hand work over to the
main thread with run_on_main() (a custom event).
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
ROOT = os.path.dirname(os.path.dirname(ADDIN_DIR))  # project folder
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

PALETTE_ID = "MechanismAssistantPalette"
CMD_ID = "MechanismAssistantCmd"
EVENT_ID = "MechanismAssistantMainThread"
MARK_GROUP = "MekanizmaAsistani"    # attribute group of the drawn document (same as fusion_lib.fourbar)
BRIDGE_PORT = 8765
TOKEN_FILE = os.path.join(ROOT, ".bridge_token")
LOG_FILE = os.path.join(ROOT, "output", "addin.log")

app = None
ui = None
handlers = []           # Fusion event handlers are kept referenced so they stay alive
pending = {}            # job id -> function to run on the main thread
pending_lock = threading.Lock()
server = None
session = {"spec": None}   # the design currently shown in the panel (K1: a sentence may edit it)


# ---------------------------------------------------------------- helpers

def log(msg):
    try:
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
        with open(LOG_FILE, "a", encoding="utf-8") as fh:
            fh.write(time.strftime("%H:%M:%S ") + msg + "\n")
    except Exception:
        pass


def run_on_main(fn):
    """From any thread: run fn on Fusion's main thread."""
    job = uuid.uuid4().hex
    with pending_lock:
        pending[job] = fn
    app.fireCustomEvent(EVENT_ID, job)


def call_on_main(fn, timeout=120):
    """run_on_main + wait for the result. Returns {'value': ...} or {'error': traceback}."""
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
        return {"error": f"Timed out ({timeout} s): Fusion may be busy."}
    return box


def reload_modules():
    """Reload the project modules: no Fusion restart needed after a code change."""
    import pipeline
    import fusion_lib.fourbar
    root = os.path.normcase(ROOT)
    ours = [m for m in list(sys.modules.values())
            if getattr(m, "__file__", None)
            and os.path.normcase(os.path.realpath(m.__file__)).startswith(root)
            and os.path.normcase(ADDIN_DIR) not in os.path.normcase(os.path.realpath(m.__file__))]
    # dependencies first: pipeline last (it imports the others)
    for mod in sorted(ours, key=lambda m: m.__name__ == "pipeline"):
        importlib.reload(mod)
    return sys.modules["pipeline"], sys.modules["fusion_lib.fourbar"]


def send_to_palette(action, payload):
    """Main thread only."""
    pal = ui.palettes.itemById(PALETTE_ID)
    if pal:
        pal.sendInfoToHTML(action, json.dumps(payload, ensure_ascii=False))


# ---------------------------------------------------------------- design flow

def start_preview(params, req):
    """Slider preview: no LLM, no drawing in Fusion; only the calculation + the panel."""
    pipeline = sys.modules.get("pipeline") or reload_modules()[0]

    def worker():
        try:
            result = pipeline.design_from_params(**params)
        except Exception as e:
            log("preview error:\n" + traceback.format_exc())
            result = {"status": "error", "message": f"Calculation error: {e}"}
        if result.get("spec"):
            session["spec"] = result["spec"]
        result["req"] = req
        run_on_main(lambda: send_to_palette("preview", result))

    threading.Thread(target=worker, daemon=True).start()


def start_band(Q, req):
    pipeline = sys.modules.get("pipeline") or reload_modules()[0]

    def worker():
        try:
            band = pipeline.swing_band(Q)
        except Exception:
            log("band error:\n" + traceback.format_exc())
            return
        run_on_main(lambda: send_to_palette("band", {"req": req, "Q": Q, "band": band}))

    threading.Thread(target=worker, daemon=True).start()


def start_fea(params):
    """Verify the hand calculation of the rocker and crank with the local FEA (gmsh + scikit-fem).
    The FEA runs in this computer's normal Python, not in Fusion's (.local_python)."""
    import subprocess
    pipeline = sys.modules.get("pipeline") or reload_modules()[0]

    def status(s):
        run_on_main(lambda: send_to_palette("status", {"text": s}))

    def worker():
        try:
            if params.get("kind", "fourbar") != "fourbar":
                raise ValueError("FEA verification is available for four-bar links only (for now).")
            r = pipeline.design_from_params(**params)
            st = r.get("strength")
            if r.get("status") != "ok" or not st:
                out = {"ok": False, "message": "The FEA needs a design that passes the checks and has a load "
                                              "(set the tip load slider above 0)."}
            else:
                import strength
                mat = strength.MATERIALS[params.get("material") or "PLA"]
                req = {"lengths": r["lengths"], "section": r["section_mm"], "load": st["load_N"],
                       "material": {"E": mat["E"], "nu": mat["nu"]},
                       "crank_torque": st["input_torque_max_Nmm"]}
                with open(os.path.join(ROOT, ".local_python"), encoding="utf-8") as fh:
                    py = fh.read().strip()
                status("Local FEA running (meshing + solving)...")
                proc = subprocess.run([py, os.path.join(ROOT, "fea_local.py"), "--json"],
                                      input=json.dumps(req), capture_output=True, text=True,
                                      timeout=300, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                if proc.returncode != 0:
                    log("FEA error:\n" + proc.stderr[-3000:])
                    out = {"ok": False, "message": "The FEA did not run (details: output/addin.log)."}
                else:
                    out = {"ok": True, "links": json.loads(proc.stdout), "material": mat["name"]}
        except ValueError as e:
            out = {"ok": False, "message": str(e)}
        except Exception as e:
            log("FEA error:\n" + traceback.format_exc())
            out = {"ok": False, "message": f"FEA error: {e}"}
        run_on_main(lambda: send_to_palette("fea", out))

    threading.Thread(target=worker, daemon=True).start()


def start_design(text=None, params=None):
    """Design from a sentence (with the LLM) or from slider values, and draw it in Fusion."""
    pipeline, fourbar = reload_modules()

    def status(s):
        run_on_main(lambda: send_to_palette("status", {"text": s}))

    def worker():
        try:
            if params is not None:
                result = pipeline.design_from_params(**params)
            else:
                result = pipeline.design_from_text(text, on_status=status, prev_spec=session["spec"])
        except urllib.error.URLError:
            result = {"status": "error",
                      "message": "Could not reach the local LLM (Ollama). Is Ollama running?"}
        except Exception as e:
            log("design error:\n" + traceback.format_exc())
            result = {"status": "error", "message": f"Unexpected error: {e}"}
        if result.get("status") == "ok":
            status("Drawing in Fusion...")
        run_on_main(lambda: finish_design(result, fourbar))

    threading.Thread(target=worker, daemon=True).start()


def finish_design(result, fourbar):
    if result.get("spec"):
        session["spec"] = result["spec"]
    if result.get("status") == "ok":
        try:
            fourbar.build(result["fusion"])
            result["drawn"] = True
        except Exception as e:
            log("drawing error:\n" + traceback.format_exc())
            result["drawn"] = False
            result["message"] += f" But drawing failed: {e}"
    send_to_palette("result", result)


# ---------------------------------------------------------------- turning (animation)

anim = {"running": False}
STEP_DEG = 4          # input angle per step
STEP_PAUSE = 0.02     # pause between steps (s)
INPUT_JOINTS = ("Ground-Pinion", "Ground-Cam", "Ground-Shaft", "Ground-Input", "Ground-Driver")   # + "Ground-Crank ..."


def crank_motion():
    """The input joint of the mechanism in the active document (None if there is none)."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        return None
    for j in design.rootComponent.asBuiltJoints:
        if j.name.startswith("Ground-Crank") or j.name in INPUT_JOINTS:
            return adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)
    return None


def cam_follower():
    """For a cam design: (follower slider joint, s table [mm, per degree], slide sign) or None."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        return None
    table = design.attributes.itemByName(MARK_GROUP, "cam_table")
    sign = design.attributes.itemByName(MARK_GROUP, "cam_slide_sign")
    joint = [j for j in design.rootComponent.asBuiltJoints if j.name == "Ground-Follower"]
    if not (table and sign and joint):
        return None
    return (adsk.fusion.SliderJointMotion.cast(joint[0].jointMotion), json.loads(table.value), float(sign.value))


def geneva_wheel():
    """For a Geneva design: (wheel joint, wheel angle table [deg per degree of driver turn], signs) or None."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        return None
    table = design.attributes.itemByName(MARK_GROUP, "geneva_table")
    signs = design.attributes.itemByName(MARK_GROUP, "geneva_signs")
    joint = [j for j in design.rootComponent.asBuiltJoints if j.name == "Ground-Wheel"]
    if not (table and signs and joint):
        return None
    return (adsk.fusion.RevoluteJointMotion.cast(joint[0].jointMotion), json.loads(table.value), json.loads(signs.value))


def spring_frames():
    """For a spring design: the compression frame components (0 = free ... last = working) or None."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or not design.attributes.itemByName(MARK_GROUP, "spring_frames"):
        return None
    occs = [o for o in design.rootComponent.occurrences if o.component.name.startswith("Spring ")]
    return sorted(occs, key=lambda o: int(o.component.name.split()[1])) or None


def animate_spring(mode):
    """Spring: compress and release by showing the frames in turn. 'turn' = one compress-release cycle."""
    anim["running"] = True
    send_to_palette("anim", {"running": True, "mode": mode})

    def worker():
        n = len(spring_frames() or [])
        seq = list(range(n)) + list(range(n - 2, 0, -1))      # 0..n-1..1 (then 0 again)
        i = 0
        while anim["running"]:
            k = seq[i % len(seq)]
            done = threading.Event()

            def show(k=k):
                try:
                    for j, occ in enumerate(spring_frames() or []):
                        occ.isLightBulbOn = (j == k)
                except Exception:
                    anim["running"] = False
                finally:
                    done.set()

            run_on_main(show)
            done.wait(2.0)
            time.sleep(0.07)
            i += 1
            if mode == "turn" and i > len(seq):
                break
        run_on_main(lambda: [setattr(o, "isLightBulbOn", j == 0) for j, o in enumerate(spring_frames() or [])])
        anim["running"] = False
        run_on_main(lambda: send_to_palette("anim", {"running": False}))

    threading.Thread(target=worker, daemon=True).start()


def rotate(mode):
    """mode: 'turn' (one full turn) or 'toggle' (continuous turning on / off)."""
    if anim["running"]:
        if mode == "toggle":
            anim["running"] = False       # the running thread stops and updates the panel
        return
    if spring_frames():
        animate_spring(mode)
        return
    if crank_motion() is None:
        send_to_palette("status", {"text": "Nothing to turn in the open document. Press 'Draw in Fusion' first."})
        return
    anim["running"] = True
    anim["geneva_phi"] = None                  # re-read from the wheel joint at the first step
    send_to_palette("anim", {"running": True, "mode": mode})

    # Limited-angle mechanism (B9): with joint limits, oscillate between them instead of a full turn
    motion0 = crank_motion()
    lim = motion0.rotationLimits
    if lim.isMinimumValueEnabled and lim.isMaximumValueEnabled:
        lo, hi, start = lim.minimumValue, lim.maximumValue, motion0.rotationValue
        # "1 turn" = start -> upper limit -> lower limit -> start; continuous = upper <-> lower
        targets = [hi, lo, start] if mode == "turn" else None
    else:
        lo = hi = None
        targets = None

    def worker():
        done, goal_i, going_up = 0, 0, True
        step_rad = math.radians(STEP_DEG)
        while anim["running"]:
            if lo is None:                        # a mechanism that can turn fully
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
                    cf = cam_follower()                # cam: follower from the s(theta) table
                    if cf:
                        slider, table, sgn = cf
                        deg = int(round(math.degrees(motion.rotationValue))) % 360
                        slider.slideValue = sgn * table[deg * len(table) // 360] / 10.0
                    gw = geneva_wheel()                # Geneva: the wheel follows its table, step by step
                    if gw:
                        wheel, table, sg = gw
                        i0 = int(round(sg["driver"] * math.degrees(motion.rotationValue - delta))) % 360
                        i1 = int(round(sg["driver"] * math.degrees(motion.rotationValue))) % 360
                        dphi = table[i1] - table[i0]
                        if abs(dphi) > sg["step"] / 2:      # the table restarts after a full turn
                            dphi -= math.copysign(sg["step"], dphi)
                        # Fusion rounds joint angles to 0.1°: keep the exact angle here, set it absolutely
                        if anim.get("geneva_phi") is None:
                            anim["geneva_phi"] = sg["wheel"] * math.degrees(wheel.rotationValue)
                        anim["geneva_phi"] += dphi
                        wheel.rotationValue = sg["wheel"] * math.radians(anim["geneva_phi"])
                except Exception:
                    anim["running"] = False       # document closed etc.
                finally:
                    stepped.set()

            run_on_main(step)
            stepped.wait(2.0)
            time.sleep(STEP_PAUSE)
        anim["running"] = False
        run_on_main(lambda: send_to_palette("anim", {"running": False}))

    threading.Thread(target=worker, daemon=True).start()


# ---------------------------------------------------------------- Fusion events

class MainThreadHandler(adsk.core.CustomEventHandler):
    def notify(self, args):
        with pending_lock:
            fn = pending.pop(args.additionalInfo, None)
        if fn:
            try:
                fn()
            except Exception:
                log("main thread error:\n" + traceback.format_exc())


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
                anim["running"] = False           # stop turning before drawing
                start_design(params=data["params"])
                html_args.returnData = "started"
            elif html_args.action == "fea":
                start_fea(data["params"])
                html_args.returnData = "started"
            elif html_args.action == "rotate":
                rotate(data.get("mode", "turn"))
                html_args.returnData = "ok"
            elif html_args.action == "ready":
                pipeline, _ = reload_modules()
                html_args.returnData = json.dumps(pipeline.MODULES, ensure_ascii=False)   # C0
        except Exception:
            log("panel error:\n" + traceback.format_exc())


class StartupCompletedHandler(adsk.core.ApplicationEventHandler):
    """Open the panel once Fusion has finished starting (a panel opened earlier vanished while the UI loaded)."""
    def notify(self, args):
        try:
            show_palette()
        except Exception:
            log("startup panel error:\n" + traceback.format_exc())


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        on_execute = CommandExecuteHandler()
        args.command.execute.add(on_execute)
        handlers.append(on_execute)


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        show_palette()


PALETTE_POS = os.path.join(ROOT, "output", "palette_pos.json")   # per machine (output/ is not in git)


def _save_palette_pos(pal):
    """Remember where the user put the floating palette (screen coordinates, any monitor)."""
    try:
        if pal.dockingState == adsk.core.PaletteDockingStates.PaletteDockStateFloating:
            os.makedirs(os.path.dirname(PALETTE_POS), exist_ok=True)
            with open(PALETTE_POS, "w", encoding="utf-8") as fh:
                json.dump({"left": pal.left, "top": pal.top, "width": pal.width, "height": pal.height}, fh)
    except Exception:
        log("could not save the panel position:\n" + traceback.format_exc())


def show_palette():
    pal = ui.palettes.itemById(PALETTE_ID)
    if not pal:
        url = os.path.join(ADDIN_DIR, "palette.html").replace("\\", "/")
        pal = ui.palettes.add(PALETTE_ID, "Mechanism Assistant", url, False, True, True, 1000, 300)
        on_html = PaletteHTMLHandler()
        pal.incomingFromHTML.add(on_html)
        handlers.append(on_html)
        # Docked at the bottom, Fusion could collapse the panel and "lose" it: a floating window instead
        pal.dockingState = adsk.core.PaletteDockingStates.PaletteDockStateFloating
        pal.setSize(1100, 320)
        # A new floating palette opens at (0, 0): on a dual-monitor setup that is the other screen.
        # Restore where the user last left it (saved in stop()); first time: near the model view bottom.
        try:
            with open(PALETTE_POS, encoding="utf-8") as fh:
                p = json.load(fh)
            pal.setSize(int(p["width"]), int(p["height"]))
            pal.setPosition(int(p["left"]), int(p["top"]))
        except Exception:
            try:
                pal.setPosition(200, max(150, app.activeViewport.height - 250))
            except Exception:
                pal.setPosition(200, 400)
    pal.isVisible = True


# ---------------------------------------------------------------- bridge (development)

TOKEN = None


def bridge_exec(code):
    """Run Python code on the main thread and return what it printed."""
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
            self._reply(403, {"error": "invalid token"})
            return
        body = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode("utf-8")
        if self.path == "/exec":
            self._reply(200, bridge_exec(body))
        elif self.path == "/screenshot":
            self._reply(200, bridge_screenshot())
        else:
            self._reply(404, {"error": "unknown path"})

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
    # On Windows SO_REUSEADDR lets two servers open the same port (if the add-in starts twice, the requests
    # go to the old server). Off: an occupied port raises an error.
    allow_reuse_address = False
    daemon_threads = True


def start_bridge():
    global server, TOKEN
    # The token is persistent: it stays the same when the add-in restarts
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
        log(f"bridge could not start (port {BRIDGE_PORT}): {e}")
        return
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log(f"bridge ready: 127.0.0.1:{BRIDGE_PORT}")


def warm_up_llm():
    """Load the model into VRAM early: the first request then takes seconds instead of ~100 s."""
    def worker():
        try:
            import llm_extract
            req = urllib.request.Request(
                "http://localhost:11434/api/generate",
                data=json.dumps({"model": llm_extract.MODEL, "keep_alive": "30m"}).encode(),
                headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=300).read()
            log("LLM warmed up")
        except Exception as e:
            log(f"LLM warm-up failed: {e}")
    threading.Thread(target=worker, daemon=True).start()


# ---------------------------------------------------------------- life cycle

RUNNING_FLAG = "_mechanism_assistant_running"   # on the sys module shared by all add-ins
owner = False                                   # did this copy start it?


def run(context):
    global app, ui, owner
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        # The same add-in may be registered twice (junction + real folder): only one copy may run
        if getattr(sys, RUNNING_FLAG, False):
            log("---- second copy skipped (the add-in is already running): " + ADDIN_DIR)
            return
        setattr(sys, RUNNING_FLAG, True)
        owner = True
        log("---- add-in started: " + os.path.realpath(__file__))

        custom_event = app.registerCustomEvent(EVENT_ID)
        on_main = MainThreadHandler()
        custom_event.add(on_main)
        handlers.append(on_main)

        cmd_def = ui.commandDefinitions.itemById(CMD_ID) or ui.commandDefinitions.addButtonDefinition(
            CMD_ID, "Mechanism Assistant", "Design mechanisms in plain language (local AI)")
        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        handlers.append(on_created)
        panel = ui.allToolbarPanels.itemById("SolidScriptsAddinsPanel")
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)

        start_bridge()
        warm_up_llm()
        if app.isStartupComplete:
            show_palette()            # Run pressed by hand: open now
        else:
            on_startup = StartupCompletedHandler()   # Fusion is starting: wait until it is done
            app.startupCompleted.add(on_startup)
            handlers.append(on_startup)
    except Exception:
        log("run error:\n" + traceback.format_exc())
        if ui:
            ui.messageBox("Mechanism Assistant could not start:\n" + traceback.format_exc())


def stop(context):
    global server, owner
    if not owner:
        return          # not the running copy: do not touch its resources
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
            _save_palette_pos(pal)
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
        log("---- add-in stopped")
    except Exception:
        log("stop error:\n" + traceback.format_exc())
