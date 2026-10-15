"""Cámara + detector. Ambos detectores devuelven la respuesta con el formato del
SDK de Edge Impulse ({"result": {"bounding_boxes": [...]}}) en coordenadas del
modelo, para que `car_core.get_detections` funcione igual con los dos.

- `BrickDetector` (UNO Q): brick `arduino:object_detection` de App Lab. El modelo
  se elige en App Lab (Brick Configuration → AI models), desde el proyecto de
  Edge Impulse; no hace falta `edge_impulse_linux`.
- `EimDetector` (laptop): `.eim` local con `edge_impulse_linux` (fomo_core).
"""
import cv2

# Etiquetas del Impulse #1 de hackaton_arduinoq3. Si el brick carga otro modelo
# (p. ej. el YOLOX de COCO por defecto) la app se niega a arrancar.
EXPECTED_LABELS = {"azul", "rojo", "verde"}


def check_labels(labels, origin):
    found = set(labels)
    if found != EXPECTED_LABELS:
        raise RuntimeError(
            f"{origin}: el modelo cargado tiene etiquetas {sorted(found)[:10]}, se esperaban "
            f"{sorted(EXPECTED_LABELS)}. No es el Impulse #1 de hackaton_arduinoq3: revisa "
            "el modelo elegido en App Lab (Brick Configuration → AI models).")


class BrickDetector:
    """Brick Object Detection de App Lab (runner HTTP del modelo de Edge Impulse).

    No está documentado en qué coordenadas devuelve las cajas, así que se le
    manda el frame ya redimensionado al tamaño de entrada del modelo: las cajas
    vuelven en coordenadas del modelo, igual que con el SDK.
    """

    resize_mode = "squash"  # cómo se redimensiona aquí; get_detections lo deshace

    def __init__(self, min_confidence=0.5):
        from arduino.app_bricks.object_detection import ObjectDetection
        self.brick = ObjectDetection(confidence=min_confidence)
        info = self.brick._model_info
        self.dim = (info.image_input_width, info.image_input_height)
        self.min_confidence = min_confidence
        print(f"Modelo (brick): {getattr(info, 'name', '?')} (proyecto {getattr(info, 'project_id', '?')})  "
              f"tipo={info.model_type}  entrada={self.dim[0]}x{self.dim[1]}  etiquetas={info.labels}")
        check_labels(info.labels, "Brick Object Detection")

    def infer(self, frame_bgr):
        small = cv2.resize(frame_bgr, self.dim, interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 90])  # BGR → JPEG correcto
        if not ok:
            return {}
        out = self.brick.detect(jpg.tobytes(), image_type="jpg", confidence=self.min_confidence)
        boxes = []
        for d in (out or {}).get("detection", []):
            x1, y1, x2, y2 = d["bounding_box_xyxy"]
            boxes.append({"label": d["class_name"],
                          "value": float(d["confidence"]) / 100.0,  # el brick da "87.50" (%)
                          "x": x1, "y": y1, "width": x2 - x1, "height": y2 - y1})
        return {"result": {"bounding_boxes": boxes}}

    def stop(self):
        pass


class EimDetector:
    """`.eim` local con edge_impulse_linux (para la laptop, build x86_64)."""

    def __init__(self, model_path, min_confidence=0.5):
        from fomo_core import load_model
        self.runner, info = load_model(model_path)
        check_labels(info["model_parameters"]["labels"], model_path)
        self.dim = self.runner.dim
        self.resize_mode = self.runner.resizeMode

    def infer(self, frame_bgr):
        from fomo_core import run_inference
        return run_inference(self.runner, frame_bgr)

    def stop(self):
        self.runner.stop()


class FrameSource:
    """Cámara: periférico Camera de App Lab en el UNO Q; OpenCV en la laptop."""

    def __init__(self, camera_id=0, resolution=(640, 480), fps=30):
        try:
            from arduino.app_peripherals.camera import Camera
            self.cam = Camera(camera_id, resolution=resolution, fps=fps)
            self.cam.start()
            self.kind = "app_lab"
        except ImportError:
            self.cam = cv2.VideoCapture(camera_id)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
            self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # menos latencia
            self.kind = "opencv"

    def read(self):
        """Frame BGR o None."""
        if self.kind == "app_lab":
            return self.cam.capture()
        ok, frame = self.cam.read()
        return frame if ok else None

    def release(self):
        if self.kind == "app_lab":
            self.cam.stop()
        else:
            self.cam.release()
