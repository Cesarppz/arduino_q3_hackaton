"""App del UNO Q: cámara → FOMO → color → BalloonController → drive(v, w) vía Bridge.

En la laptop corre igual sin `arduino.app_utils`: imprime los comandos y,
con SHOW=1, abre una ventana con el overlay.

Variables de entorno:
    EI_MODEL       ruta del .eim (por defecto model.eim junto a este archivo)
    CAMERA_ID      índice de la cámara (0)
    TARGET_COLOR   rojo | verde | azul. Si no se define en el UNO Q, se elige
                   con el botón del sketch (pulsación corta = color, larga = arrancar)
    CAR_MODE       run (por defecto) | stop_sweep | straight | spin  (ver calibrate.py)
    SHOW           1 = ventana con overlay (solo laptop)
"""
import os
import time

import cv2

from car_core import (COLORS, BalloonController, BridgeDrive, CarConfig, DriveSender, PrintDrive,
                      classify_detections, draw_car_overlay, send_calibration)
from fomo_core import get_centroids, load_model, run_inference

HERE = os.path.dirname(os.path.abspath(__file__))

# Calibración de los servos (copiar aquí lo obtenido con calibrate.py).
# STOP_x: µs en que el servo no gira · K_x_*: µs por unidad de velocidad ·
# DB_x: µs extra para salir de la zona muerta del servo.
SERVO_CAL = {
    "STOP_L": 1500, "STOP_R": 1500,
    "K_L_FWD": 2.0, "K_L_REV": 2.0,
    "K_R_FWD": 2.0, "K_R_REV": 2.0,
    "DB_L": 20, "DB_R": 20,
}

cfg = CarConfig(target_color=os.environ.get("TARGET_COLOR", "rojo"))
MODE = os.environ.get("CAR_MODE", "run")
SHOW = os.environ.get("SHOW") == "1"

try:
    from arduino.app_utils import App
    ON_UNO_Q = True
except ImportError:
    ON_UNO_Q = False

transport = BridgeDrive() if ON_UNO_Q else PrintDrive()
send_calibration(transport, SERVO_CAL)
sender = DriveSender(transport, heartbeat_s=0.1)


def wait_for_start():
    """Con TARGET_COLOR arranca ya; si no (UNO Q), espera la pulsación larga del botón."""
    if not ON_UNO_Q or "TARGET_COLOR" in os.environ:
        transport.set_running(1)
        return
    print("Esperando botón: corta = cambiar color, larga = arrancar")
    while True:
        idx = transport.get_color()
        if idx >= 0:
            cfg.target_color = COLORS[idx]
            return
        time.sleep(0.1)


if MODE != "run":
    import calibrate
    transport.set_running(1)
    loop = calibrate.make_loop(MODE, transport, sender)
else:
    runner, _ = load_model(os.environ.get("EI_MODEL", os.path.join(HERE, "model.eim")))
    model_w, model_h = runner.dim
    cam = cv2.VideoCapture(int(os.environ.get("CAMERA_ID", 0)))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # menos latencia: siempre el frame más nuevo
    wait_for_start()
    print(f"Objetivo: {cfg.pops_goal} globos {cfg.target_color}")
    ctrl = BalloonController(cfg)
    last = {"state": None, "t": time.monotonic()}

    def loop():
        ok, frame = cam.read()
        if not ok:
            sender.send(0, 0)
            time.sleep(0.1)
            return
        h, w = frame.shape[:2]
        t0 = time.perf_counter()
        res = run_inference(runner, frame)
        cents = get_centroids(res, w, h, model_w, model_h, runner.resizeMode, cfg.min_confidence)
        dets = classify_detections(frame, cents, cfg)
        out = ctrl.step(dets, w, h)
        sender.send(out["v"], out["w"])
        dt_ms = (time.perf_counter() - t0) * 1000

        if out["state"] != last["state"]:
            print(f"[{out['state']:8s}] globos={out['popped']}/{cfg.pops_goal} "
                  f"objetivos={len(out['targets'])} otros={len(out['obstacles'])} ({dt_ms:.0f} ms)")
            last["state"] = out["state"]
        if SHOW:
            now = time.monotonic()
            fps = 1 / max(now - last["t"], 1e-3)
            last["t"] = now
            cv2.imshow("balloon car", draw_car_overlay(frame, out, cfg, fps))
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
            runner.stop()
