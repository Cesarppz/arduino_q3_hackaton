"""App del UNO Q: cámara → YOLO (brick Object Detection) [+ VitTrack] → BalloonController → drive(v, w) vía Bridge.

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

# Modo híbrido: YOLO detecta, VitTrack sigue al globo y YOLO lo re-verifica
# cada VERIFY_EVERY_S segundos. False = YOLO en cada frame.
USE_TRACKER = True
VERIFY_EVERY_S = 3.0
# El int8bq es más rápido pero necesita una OpenCV reciente; si no carga, usar el fp32.
TRACKER_MODEL = "object_tracking_vittrack_2023sep.onnx"
# ==============================================

cfg = CarConfig(target_color=TARGET_COLOR, use_tracker=USE_TRACKER, verify_every=VERIFY_EVERY_S)
MODE = os.environ.get("CAR_MODE", "run")
SHOW = os.environ.get("SHOW") == "1"

try:
    from arduino.app_utils import App
    ON_UNO_Q = True
except ImportError:
    ON_UNO_Q = False

transport = BridgeDrive() if ON_UNO_Q else PrintDrive()
send_calibration(transport, MOTOR_CAL)
sender = DriveSender(transport, heartbeat_s=0.1)


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
    last = {"state": None, "t": time.monotonic()}

    def loop():
        frame = cam.read()
        if frame is None:
            sender.send(0, 0)
            time.sleep(0.1)
            return
        h, w = frame.shape[:2]
        dets = perception.get(frame, ctrl)
        out = ctrl.step(dets, w, h)
        sender.send(out["v"], out["w"])
        perception.after_step(frame, out, ctrl)

        if out["state"] != last["state"]:
            print(f"[{out['state']:8s}] globos={out['popped']}/{cfg.pops_goal} "
                  f"objetivos={len(out['targets'])} otros={len(out['obstacles'])} "
                  f"({perception.mode} {perception.ms:.0f} ms)")
            last["state"] = out["state"]
        if SHOW:
            now = time.monotonic()
            fps = 1 / max(now - last["t"], 1e-3)
            last["t"] = now
            cv2.imshow("balloon car", draw_car_overlay(frame, out, cfg, fps, perception.status()))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt


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
            cam.release()
            perception.stop()
            detector.stop()
