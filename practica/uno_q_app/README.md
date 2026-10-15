# FOMO Tracker para Arduino UNO Q

Cámara USB → modelo FOMO (Edge Impulse, `.eim`) en el Linux del UNO Q → byte de flags → STM32 vía **Bridge** → stepper.

```
uno_q_app/
├── app.yaml
├── python/
│   ├── main.py          # loop de App Lab (App.run) + Bridge.call("set_flags", byte)
│   ├── fomo_core.py     # lógica compartida con el notebook fomo_desviacion.ipynb
│   ├── requirements.txt
│   └── model.eim        # ← copia aquí tu modelo (build Linux AARCH64)
└── sketch/
    └── sketch.ino       # Bridge.provide_safe("set_flags") + stepper no bloqueante
```

## Protocolo (1 byte)
| bit | flag | significado |
|---|---|---|
| 0 | DETECTED | hay objetivo |
| 1 | LEFT | objeto a la izquierda del centro |
| 2 | RIGHT | objeto a la derecha |
| 3 | UP | objeto arriba |
| 4 | DOWN | objeto abajo |
| 5 | CENTERED | dentro de la zona muerta en ambos ejes |
| 6 | HOLD | objeto perdido momentáneamente, se usa la última posición |

Python envía el byte solo cuando cambia y además cada 200 ms (latido). Si el sketch pasa 500 ms sin recibir nada, detiene el motor.

## Pasos
1. **Modelo:** en Edge Impulse Studio → *Deployment* → **Linux (AARCH64)** → descarga el `.eim` y cópialo como `python/model.eim`. El de la laptop (x86_64) **no** corre en el UNO Q; `load_model` avisa si te equivocas.
2. **App Lab:** crea una app nueva y copia dentro `python/` y `sketch/`. Si App Lab genera su propio `app.yaml` o `sketch.yaml`, quédate con los suyos.
3. Conecta la cámara USB (con hub USB-C con alimentación) y el driver ULN2003 a los pines 8, 9, 10 y 11.
4. **Run.** En la consola de Python verás `DETECTED|LEFT ... dx=-12.3°`.

## Probar en la laptop
```bash
cd practica/uno_q_app/python
EI_MODEL=../../cesar727-project-1-linux-x86_64-v2-fomo.eim ../../../.venv/bin/python main.py
# con un Arduino clásico conectado y arduino_serial_flags.ino cargado:
FLAGS_SERIAL_PORT=/dev/ttyACM0 EI_MODEL=... ../../../.venv/bin/python main.py
```

## Cosas sin verificar (revisar en la placa)
- Que `pip` instale `requirements.txt` en el entorno de App Lab y que el contenedor tenga acceso a `/dev/video0`. Si no, usa el brick `video_object_detection` con tu modelo (`EI_OBJ_DETECTION_MODEL` en `app.yaml`).
- El sketch no se compiló contra el core Zephyr del UNO Q. Solo usa `Bridge.begin`, `Bridge.provide_safe`, `digitalWrite` y `micros`.
- Si el motor gira al revés, cambia el signo de `dir` en `sketch.ino`.
