from arduino.app_utils import App
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from arduino.app_utils.bridge import Bridge
from datetime import datetime, UTC

ui = WebUI()
detection_stream = VideoObjectDetection(confidence=0.5, debounce_sec=0.0)

# Variable global para recordar qué fue lo último que le dijimos al robot
# Vital para no saturar el Bridge enviando el mismo texto 30 veces por segundo
last_command = ""

def send_detections_to_ui(detections: dict):
    global last_command
    
    # 1. ESTADO POR DEFECTO (Failsafe)
    # Si la cámara se tapa o el objeto desaparece, la acción segura es detenerse.
    accion = "STOP"
    velocidad = 0

    # 2. TOMA DE DECISIONES BASADA EN VISIÓN
    if detections:
        for key, values in detections.items():
            if key == "cat":
                accion = "GO"
                velocidad = 60
                break # Encontramos nuestro objetivo, dejamos de buscar otros
            
            elif key == "dog":
                accion = "LEFT"
                velocidad = 45
                break
                
            # Si quieres implementar el PID en el futuro, aquí calcularías el error 
            # de la posición 'x' del objeto y se lo asignarías a 'velocidad'.

    # 3. EMPAQUETADO DEL COMANDO
    # Creamos el string exacto que tu función de C++ espera recibir: "ACCION VELOCIDAD"
    current_command = f"{accion} {velocidad}"

    # 4. ENVÍO SEGURO AL BRIDGE
    # Solo "despertamos" al C++ si la orden es diferente a la anterior
    if current_command != last_command:
        # Usamos "Robot" porque así lo bautizaste en el Bridge.provide_safe de C++
        Bridge.call("Robot", current_command)
        last_command = current_command

    # 5. ACTUALIZACIÓN DE LA INTERFAZ WEB (Reciclado de tu código original)
    for key, values in detections.items():
        for value in values:
            entry = {
                "content": key,
                "confidence": value.get("confidence"),
                "timestamp": datetime.now(UTC).isoformat()
            }
            ui.send_message("detection", message=entry)

detection_stream.on_detect_all(send_detections_to_ui)

App.run()
