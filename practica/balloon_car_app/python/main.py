"""App del UNO Q: cámara → YOLO "globo" (brick Object Detection) → color HSV [+ VitTrack] → BalloonController → drive(v, w) vía Bridge.

YOLO por el brick tarda ~0.7 s por frame y el sketch para los motores a los
300 ms sin comando, así que la percepción va en un hilo aparte y el control
corre a 20 Hz (CONTROL_DT): entre frames el controlador sigue con el último
comando y los estados con tiempo (embestida, retroceso, pulsos) avanzan con el reloj.

En el UNO Q el modelo lo sirve el brick `arduino:object_detection` (se elige en
App Lab: Brick Configuration → AI models, desde el proyecto de Edge Impulse).
En la laptop corre igual sin `arduino.app_utils` con un `.eim` x86_64 local:
imprime los comandos y, con SHOW=1, abre una ventana con el overlay.

Variables de entorno (opcionales; lo normal es editar las constantes de abajo):
    EI_MODEL       solo laptop: ruta del .eim (por defecto model.eim junto a este archivo)
    CAMERA_ID      índice de la cámara (0)
    TARGET_COLOR   rojo | verde | azul
    CAR_MODE       run (por defecto) | min_sweep | straight | spin  (ver calibrate.py)
    SHOW           1 = ventana con overlay (solo laptop)
"""
import os
import threading
import time

import cv2

from car_core import (BalloonController, BridgeDrive, CarConfig, DriveSender, PrintDrive,
                      draw_car_overlay, send_calibration)
from tracker import BalloonTracker, HybridPerception
from vision import BrickDetector, EimDetector, FrameSource

HERE = os.path.dirname(os.path.abspath(__file__))

# ======== AJUSTAR ANTES DE CADA SALIDA ========
TARGET_COLOR = os.environ.get("TARGET_COLOR", "rojo")   # rojo | verde | azul
WAIT_BUTTON = False   # True = no arranca hasta pulsar el botón (D2 a GND)

# Compensación de los motores DC (copiar aquí lo obtenido con calibrate.py):
#   GAIN_x: % de la velocidad pedida (bajar la del motor que va más rápido)
#   MIN_x : % a partir del cual la rueda empieza a girar
MOTOR_CAL = {"GAIN_IZQ": 100, "GAIN_DER": 100, "MIN_IZQ": 20, "MIN_DER": 20}

# Sentido del giro: w > 0 debe girar a la DERECHA. Este carro giraba al revés
# (globo a la izquierda → giraba a la derecha), por eso -1. Poner 1 si cambia el cableado.
TURN_SIGN = -1

# Modo híbrido: YOLO detecta, VitTrack sigue al globo y YOLO lo re-verifica
# cada VERIFY_EVERY_S segundos. False = YOLO en cada frame.
# En la placa no funciona: su OpenCV 4.13 viene sin el módulo dnn que usa VitTrack.
USE_TRACKER = False
VERIFY_EVERY_S = 3.0
# El int8bq es más rápido pero necesita una OpenCV reciente; si no carga, usar el fp32.
TRACKER_MODEL = "object_tracking_vittrack_2023sep.onnx"
CONTROL_DT = 0.05     # s entre comandos a los motores (20 Hz)
# ==============================================

cfg = CarConfig(target_color=TARGET_COLOR, use_tracker=USE_TRACKER, verify_every=VERIFY_EVERY_S)
MODE = os.environ.get("CAR_MODE", "run")
SHOW = os.environ.get("SHOW") == "1"

try:
    from arduino.app_utils import App
    ON_UNO_Q = True
except ImportError:
    ON_UNO_Q = False

transport = BridgeDrive(TURN_SIGN) if ON_UNO_Q else PrintDrive(TURN_SIGN)
send_calibration(transport, MOTOR_CAL)
sender = DriveSender(transport, heartbeat_s=0.1)


class PerceptionThread:
    """Cámara + YOLO/tracker en un hilo; el control toma el último resultado sin esperar."""

    def __init__(self, cam, perception, ctrl):
        self.cam, self.perception, self.ctrl = cam, perception, ctrl
        self.lock = threading.Lock()
        self.latest = None               # (frame, dets, t_frame) aún no consumido
        self.consumed = threading.Event()
        self.out = None
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            t_frame = time.monotonic()
            frame = self.cam.read()
            if frame is None:
                time.sleep(0.05)
                continue
            try:
                dets = self.perception.get(frame, self.ctrl)
            except Exception as e:  # noqa: BLE001 — un fallo del brick no debe matar el hilo
                print(f"Error de percepción: {e}")
                time.sleep(0.2)
                continue
            self.consumed.clear()
            with self.lock:
                self.latest = (frame, dets, t_frame)
            # el tracker se arranca con la salida del control para este mismo frame
            if self.consumed.wait(1.0) and self.out is not None:
                self.perception.after_step(frame, self.out, self.ctrl)

    def take(self):
        with self.lock:
            item, self.latest = self.latest, None
        return item

    def done(self, out):
        self.out = out
        self.consumed.set()

    def stop(self):
        self.running = False
        self.consumed.set()
        self.thread.join(timeout=3)


def wait_for_start():
    if not (ON_UNO_Q and WAIT_BUTTON):
        transport.set_running(1)
        return
    print("Esperando el botón para arrancar…")
    while not transport.get_running():
        time.sleep(0.1)


if MODE != "run":
    import calibrate
    transport.set_running(1)
    loop = calibrate.make_loop(MODE, transport, sender)
else:
    if ON_UNO_Q:
        detector = BrickDetector(cfg.min_confidence)
    else:
        detector = EimDetector(os.environ.get("EI_MODEL", os.path.join(HERE, "model.eim")))
    tracker = None
    if cfg.use_tracker:
        try:
            tracker = BalloonTracker(os.path.join(HERE, TRACKER_MODEL), cfg.track_min_score)
            print(f"Tracker VitTrack: {TRACKER_MODEL} (verificación YOLO cada {cfg.verify_every:.1f} s)")
        except Exception as e:  # noqa: BLE001 — sin tracker la app sigue con YOLO
            print(f"AVISO: tracker desactivado, solo YOLO ({e})")
    perception = HybridPerception(detector, cfg, tracker)
    cam = FrameSource(int(os.environ.get("CAMERA_ID", 0)))
    wait_for_start()
    print(f"Objetivo: {cfg.pops_goal} globos {cfg.target_color}")
    ctrl = BalloonController(cfg)
    percept = PerceptionThread(cam, perception, ctrl)
    last = {"state": None, "t": time.monotonic(), "size": (640, 480)}

    def loop():
        t0 = time.monotonic()
        item = percept.take()
        if item is None:
            out = ctrl.step(None, *last["size"], now=t0)
        else:
            frame, dets, t_frame = item
            h, w = frame.shape[:2]
            last["size"] = (w, h)
            out = ctrl.step(dets, w, h, now=t0, t_frame=t_frame)
            percept.done(out)
        sender.send(out["v"], out["w"])

        if out["state"] != last["state"]:
            print(f"[{out['state']:8s}] globos={out['popped']}/{cfg.pops_goal} "
                  f"objetivos={len(out['targets'])} otros={len(out['obstacles'])} "
                  f"v={out['v']:+.0f} w={out['w']:+.0f} ({perception.mode} {perception.ms:.0f} ms)")
            last["state"] = out["state"]
        if out["note"]:  # qué decidió con este frame y por qué
            print(f"    {out['note']}")
        if SHOW and item is not None:
            fps = 1 / max(t0 - last["t"], 1e-3)
            last["t"] = t0
            cv2.imshow("balloon car", draw_car_overlay(frame, out, cfg, fps, perception.status()))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt
        time.sleep(max(0.0, CONTROL_DT - (time.monotonic() - t0)))


if __name__ == "__main__":
    try:
        if ON_UNO_Q:
            App.run(user_loop=loop)
        else:
            while True:
                loop()
    except KeyboardInterrupt:
        pass
    finally:
        transport.drive(0, 0)
        if MODE == "run":
            percept.stop()
            cam.release()
            perception.stop()
            detector.stop()
