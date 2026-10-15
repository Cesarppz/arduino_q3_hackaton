"""App del Arduino UNO Q: cámara USB → FOMO (.eim) → byte de flags → STM32 vía Bridge.

En el UNO Q corre dentro de App Lab (usa `arduino.app_utils`).
En la laptop se puede probar igual: sin `arduino.app_utils` imprime los flags,
o los manda por serial a un Arduino clásico si se define FLAGS_SERIAL_PORT.

Variables de entorno opcionales:
    EI_MODEL           ruta del .eim (por defecto: model.eim junto a este archivo)
    CAMERA_ID          índice de la cámara (por defecto 0)
    FLAGS_SERIAL_PORT  p. ej. /dev/ttyACM0 (solo modo laptop)
"""
import os
import time

import cv2

from fomo_core import (BridgeTransport, Config, FlagSender, Pipeline, PrintTransport,
                       SerialTransport, decode_flags, load_model)

HERE = os.path.dirname(os.path.abspath(__file__))

cfg = Config(
    model_path=os.environ.get("EI_MODEL", os.path.join(HERE, "model.eim")),
    camera_id=int(os.environ.get("CAMERA_ID", 0)),
    min_confidence=0.5,
    target_label=None,
    deadzone=0.10,
    hysteresis=0.03,
)

try:
    from arduino.app_utils import App
    ON_UNO_Q = True
except ImportError:
    ON_UNO_Q = False

runner, _ = load_model(cfg.model_path)
pipeline = Pipeline(runner, cfg)

if ON_UNO_Q:
    transport = BridgeTransport("set_flags")
elif os.environ.get("FLAGS_SERIAL_PORT"):
    transport = SerialTransport(os.environ["FLAGS_SERIAL_PORT"])
else:
    transport = PrintTransport()
sender = FlagSender(transport, heartbeat_s=0.2)

cam = cv2.VideoCapture(cfg.camera_id)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def loop():
    ok, frame = cam.read()
    if not ok:
        sender.send(0)  # sin cámara → el MCU detiene el motor
        time.sleep(0.1)
        return
    out = pipeline.process(frame)
    if sender.send(out["flags"]) and out["deviation"]:
        d = out["deviation"]
        print(f"{decode_flags(out['flags']):30s} dx={d['dx_deg']:+.1f}° dy={d['dy_deg']:+.1f}° "
              f"({out['infer_s'] * 1000:.0f} ms)")


if __name__ == "__main__":
    try:
        if ON_UNO_Q:
            App.run(user_loop=loop)
        else:
            while True:
                loop()
    finally:
        cam.release()
        runner.stop()
