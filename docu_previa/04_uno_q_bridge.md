# 04 · Arduino UNO Q y Bridge

## La placa
El **Arduino UNO Q** tiene dos procesadores:
| Lado | Chip | Sistema | Qué corre aquí |
|---|---|---|---|
| MPU | Qualcomm Dragonwing QRB2210 | Debian Linux | `python/main.py`, el `.eim`, OpenCV, la cámara USB |
| MCU | STM32U585 | Zephyr (core Arduino) | `sketch/sketch.ino`, control del motor |

Se programan juntos desde **Arduino App Lab**: una "app" es una carpeta con `app.yaml`, `python/` y `sketch/`. Al darle *Run*, App Lab lanza el Python, flashea el sketch y arranca todo.

## Bridge (RPC entre Python y el sketch)
Bridge es la capa RPC de Arduino (sobre `arduino-router`) que conecta ambos lados.

**Python** (`from arduino.app_utils import *`):
| Llamada | Uso |
|---|---|
| `Bridge.call("nombre", args...)` | Llama a una función del sketch y espera la respuesta |
| `Bridge.notify("nombre", args...)` | Igual pero sin esperar respuesta |
| `Bridge.provide("nombre", fn)` | Expone una función de Python al sketch |
| `App.run(user_loop=loop)` | Arranca la app y llama a `loop()` repetidamente |

**Sketch** (`#include <Arduino_RouterBridge.h>`):
| Llamada | Uso |
|---|---|
| `Bridge.begin()` | Inicializa (en `setup()`) |
| `Bridge.provide("nombre", fn)` | Expone una función; se ejecuta en **otro hilo** |
| `Bridge.provide_safe("nombre", fn)` | Igual, pero se ejecuta en el **loop principal** (thread-safe) ← usamos esta |
| `Bridge.call("nombre", args...).result(var)` | Llama a Python y lee el resultado |
| `Bridge.notify("nombre", args...)` | Llama a Python sin esperar |
| `Monitor.print(...)` | Imprimir (en lugar de `Serial`) |

### ⚠ Reglas importantes
1. **No** llamar `Bridge.call` ni imprimir dentro de un handler de `provide`: la documentación advierte de *deadlock*. Nuestro `set_flags` solo guarda el byte.
2. El sketch puede arrancar **antes** que Python. Por eso el motor arranca detenido y se detiene si pasan 500 ms sin mensajes.
3. Los nombres y tipos de argumentos deben coincidir en ambos lados (`set_flags(int)` ↔ `Bridge.call("set_flags", int(flags))`).

## Nuestra app (`practica/uno_q_app/`)
### `python/main.py`
1. Crea `Config` (modelo en `EI_MODEL` o `python/model.eim`).
2. `load_model` verifica que el `.eim` sea **aarch64**.
3. Detecta el entorno: si existe `arduino.app_utils` usa `BridgeTransport`; si no (laptop), usa `SerialTransport` si existe `FLAGS_SERIAL_PORT` y, si tampoco, `PrintTransport`.
4. Cámara a 640×480. En cada `loop()`: frame → `Pipeline.process` → `FlagSender.send`.

Variables de entorno:
| Variable | Por defecto | Uso |
|---|---|---|
| `EI_MODEL` | `python/model.eim` | Ruta del modelo |
| `CAMERA_ID` | `0` | Índice de la cámara |
| `FLAGS_SERIAL_PORT` | — | Solo en laptop: enviar a un Arduino por USB |

### `sketch/sketch.ino`
- `set_flags(int)` registrado con `provide_safe`: guarda `flags` y `lastMsgMs`.
- `loop()`:
  - LED integrado encendido si `DETECTED`.
  - Si `DETECTED` y no `CENTERED`: un paso cada `STEP_INTERVAL_US` (10 RPM) hacia `LEFT` (−1) o `RIGHT` (+1).
  - En cualquier otro caso, o con timeout: bobinas apagadas (no se calienta).
- Stepper 28BYJ-48 + ULN2003 en los pines **8, 10, 9, 11** (mismo orden que `practica_stepper.ino`), con la secuencia de 4 pasos de la librería `Stepper` implementada a mano.
- `UP`/`DOWN` quedan libres para un segundo motor (tilt).

### Conexión del motor
| ULN2003 | Pin Arduino |
|---|---|
| IN1 | 8 |
| IN2 | 9 |
| IN3 | 10 |
| IN4 | 11 |
| + / − | 5 V / GND (preferible fuente externa) |

## Pendiente de verificar en la placa
- Que App Lab instale `python/requirements.txt` (`edge_impulse_linux`, `opencv-python-headless`, `numpy`).
- Que el contenedor de la app tenga acceso a `/dev/video0`.
- Que el sketch compile en el core Zephyr (solo usa `Bridge.begin`, `Bridge.provide_safe`, `pinMode`, `digitalWrite`, `millis`, `micros`).

**Plan B:** si no se puede abrir la cámara o instalar el SDK, usar el brick `video_object_detection` de App Lab con el modelo propio (variable `EI_OBJ_DETECTION_MODEL` en `app.yaml`) y reutilizar desde `fomo_core.py` solo `compute_deviation` + `DirectionEncoder` + `BridgeTransport`.

## Fuentes
- [UNO Q User Manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/)
- [Arduino_RouterBridge (GitHub)](https://github.com/arduino-libraries/Arduino_RouterBridge)
- [Foro Arduino – ejemplos de Bridge](https://forum.arduino.cc/t/arduino-uno-q-and-bridge-tool-any-samples-and-documentation/1410551)
- [Edge Impulse en Arduino App Lab](https://docs.edgeimpulse.com/hardware/deployments/run-arduino-app-lab)
- [DesignSpark – UNO Q con modelo personalizado de Edge Impulse](https://www.rs-online.com/designspark/automating-pcb-inspection-with-arduino-uno-q-part-4-creating-the-application)
