# 06 · Guía para el día del hackatón

## Checklist de material
- [ ] Arduino UNO Q + fuente / hub USB-C con alimentación
- [ ] Cámara USB
- [ ] Motor 28BYJ-48 + driver ULN2003 + cables (pines 8, 9, 10, 11)
- [ ] (Plan B) Arduino UNO clásico + cable USB
- [ ] Laptop con el `.venv` del proyecto
- [ ] El objeto a detectar (audífonos) y fondo/iluminación del lugar

## Paso a paso
### 1. Modelo (lo primero, es lo que más tarda)
1. Recolectar fotos del lugar real con `collect_dataset` (utilidad E del notebook), unas 50–100 fotos variando posición y distancia.
2. Subirlas: `edge-impulse-uploader --category split dataset/audifono/*.jpg`
3. Etiquetar en *Data acquisition → Labeling queue*.
4. *Create impulse*: imagen **96×96 o 160×160**, bloque *Object Detection* → **FOMO**.
5. Entrenar y revisar en *Model testing*.
6. *Deployment*: descargar **Linux (x86_64)** para la laptop y **Linux (AARCH64)** para el UNO Q.

### 2. Probar en la laptop
1. En el notebook, `cfg.model_path` = el `.eim` x86_64 nuevo y `RUN_CAMERA = True`.
2. `benchmark` → confirmar FPS aceptables (>10).
3. `snapshot` → ver que detecta y que la desviación tiene sentido (derecha = `dx` positivo).
4. `calibrate_live` → ajustar `deadzone`, `hysteresis`, `min_confidence` y `smoothing`. Copiar los valores a `Config` en el notebook **y** en `uno_q_app/python/main.py`.

### 3. Probar el motor sin el UNO Q (opcional)
1. Cargar `practica/arduino_serial_flags/arduino_serial_flags.ino` en un Arduino clásico.
2. `live(cfg, SerialTransport("/dev/ttyACM0"))` en el notebook, o:
   ```bash
   cd practica/uno_q_app/python
   FLAGS_SERIAL_PORT=/dev/ttyACM0 EI_MODEL=<modelo_x86>.eim ../../../.venv/bin/python main.py
   ```
3. Si el motor gira hacia el lado contrario, invertir el signo de `dir` en el sketch.

### 4. Llevarlo al UNO Q
1. Copiar el `.eim` AARCH64 a `uno_q_app/python/model.eim`.
2. En App Lab: nueva app → copiar `python/` y `sketch/` (usar el `app.yaml` que genere App Lab si da problemas).
3. Conectar cámara y motor. *Run*.
4. En la consola: líneas como `DETECTED|LEFT  dx=-12.3° dy=+1.0° (85 ms)`.

## Utilidades del notebook
| # | Utilidad | Cuándo usarla |
|---|---|---|
| A | `benchmark` | Al recibir un modelo nuevo: decide si la resolución es viable |
| B | `set_model_threshold` | El modelo detecta poco (bajar) o da falsos positivos (subir) |
| C | `simulate_mcu` | Explicar el protocolo o depurar sin motor |
| D | `steps_for_angle` | Si se quiere un movimiento proporcional en vez de "girar mientras" |
| E | `collect_dataset` | Primera hora del hackatón: fotos del lugar real |
| F | `calibrate_live` | Cuando ya detecta: ajustar la sensibilidad del motor |
| G | `record_demo` | Antes de presentar: video de respaldo |

## Ajuste fino
| Quiero… | Cambiar |
|---|---|
| Que el motor se detenga antes (más tolerante) | subir `deadzone` |
| Que deje de oscilar en el borde | subir `hysteresis` |
| Movimiento más suave | bajar `smoothing` |
| Que reaccione más rápido | subir `smoothing` |
| Que no se detenga cuando FOMO parpadea | subir `lost_hold_frames` |
| Que no salte entre dos objetos | `target_strategy="closest_prev"` |
| Que gire más rápido | subir `RPM` en el sketch (28BYJ-48: máx. ~15) |

## Troubleshooting
| Síntoma | Causa probable | Solución |
|---|---|---|
| `El .eim es para x86_64 pero esta máquina es aarch64` | Modelo de laptop en el UNO Q | Descargar build *Linux (AARCH64)* |
| `PermissionError` al cargar | `.eim` sin permiso de ejecución | `load_model` lo arregla; si no, `chmod +x` |
| `ModuleNotFoundError: pyaudio` | El SDK importa audio | Importar `fomo_core` (trae el stub) antes que `edge_impulse_linux` |
| Pocos FPS | Modelo de entrada grande | Reentrenar a 96×96 / 160×160 |
| Nunca detecta | Umbral alto o modelo no generaliza | Utilidad B, bajar `min_confidence`, más fotos del lugar (E) |
| Motor tiembla en el centro | Zona muerta chica | Utilidad F |
| Motor gira al revés | Montaje / orden de bobinas | Invertir `dir` en el sketch |
| Motor no se mueve pero hay `LEFT`/`RIGHT` | Alimentación insuficiente o cables | Fuente externa de 5 V para el ULN2003; revisar pines |
| La cámara no abre | Otro proceso la usa / índice | Cerrar otros notebooks; `camera_id=1` |
| UNO Q se congela | `Bridge.call`/print dentro de un handler | El handler solo guarda el valor |
| El motor se mueve solo al encender | El sketch arranca antes que Python | Cubierto: timeout de 500 ms |
| Serial: el primer byte se pierde | El Arduino se reinicia al abrir el puerto | `SerialTransport` ya espera 2 s |

## Guion corto para la presentación
1. **Problema:** seguir un objeto con una cámara en un dispositivo edge.
2. **Modelo:** FOMO de Edge Impulse, que da el centroide del objeto y es rápido y ligero.
3. **Cálculo:** desviación respecto al centro en píxeles y grados, con zona muerta e histéresis.
4. **Protocolo:** 1 byte de flags, enviado solo al cambiar y con latido; el MCU tiene timeout de seguridad.
5. **Hardware:** UNO Q con Linux (visión) y STM32 (motor) comunicados por Bridge.
6. **Demo** en vivo (y el video de `record_demo` como respaldo).
