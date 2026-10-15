# 05 · Referencia de `fomo_core.py`

Archivo: `practica/uno_q_app/python/fomo_core.py`. Al importarse, si falta `pyaudio` se registra un módulo vacío, porque `edge_impulse_linux` lo importa aunque no usemos audio.

## `Config` (dataclass)
| Campo | Default | Descripción |
|---|---|---|
| `model_path` | `"modelo.eim"` | Ruta del `.eim` |
| `camera_id` | `0` | Índice para `cv2.VideoCapture` |
| `min_confidence` | `0.5` | Confianza mínima de un centroide |
| `target_label` | `None` | Filtrar por etiqueta (`None` = todas) |
| `target_strategy` | `"closest_prev"` | `"confidence"`, `"closest_center"` o `"closest_prev"` |
| `deadzone` | `0.10` | Zona muerta (fracción de media imagen) |
| `hysteresis` | `0.03` | Margen para salir de un estado de dirección |
| `smoothing` | `0.5` | α de la EMA (1.0 = sin suavizado) |
| `lost_hold_frames` | `5` | Frames que se mantiene la última posición |
| `hfov_deg` / `vfov_deg` | `60.0` / `45.0` | Campo de visión de la cámara |

## Carga del modelo
| Función | Descripción |
|---|---|
| `eim_architecture(path) → str` | Lee la cabecera ELF: `"x86_64"`, `"aarch64"`, `"armv7l"`. Lanza `ValueError` si no es ELF |
| `load_model(path, verbose=True) → (runner, model_info)` | Verifica existencia y arquitectura (`RuntimeError` si no coincide), pone permiso de ejecución, inicia el runner, imprime info y avisa si no es FOMO |

## Inferencia y centroides
| Función | Descripción |
|---|---|
| `model_to_frame(x, y, frame_w, frame_h, model_w, model_h, mode) → (x, y)` | Coordenadas del modelo → frame según el resize mode |
| `run_inference(runner, frame_bgr) → dict` | BGR→RGB, features con la configuración de Studio, `classify` |
| `get_centroids(res, frame_w, frame_h, model_w, model_h, mode, min_confidence, target_label) → list` | Lista de `{"label", "confidence", "x", "y"}` en píxeles del frame |
| `pick_target(centroids, frame_w, frame_h, strategy, prev) → dict \| None` | Un único objetivo |

## Filtro
`CentroidFilter(smoothing=0.5, hold_frames=5)`
| Miembro | Descripción |
|---|---|
| `update(target) → ((x, y), held) \| (None, False)` | EMA si hay objetivo; si no, devuelve la última posición con `held=True` hasta `hold_frames` veces |
| `pos` | Última posición filtrada (la usa `closest_prev`) |
| `alpha`, `hold_frames`, `missed` | Estado interno |

## Desviación
`compute_deviation(centroid, frame_w, frame_h, hfov_deg=60, vfov_deg=45) → dict` con `dx_px, dy_px, dx_norm, dy_norm, dx_deg, dy_deg` (x+ derecha, y+ arriba).

## Codificación
| Símbolo | Descripción |
|---|---|
| `DETECTED, LEFT, RIGHT, UP, DOWN, CENTERED, HOLD` | Máscaras de bits (`1 << 0` … `1 << 6`) |
| `FLAG_NAMES` | `{máscara: nombre}` |
| `DirectionEncoder(deadzone, hysteresis).encode(deviation, held=False) → int` | Byte de flags; `deviation=None` → `0` y reinicia estados |
| `decode_flags(flags) → str` | `"DETECTED\|LEFT"`, o `"NONE"` |

## Pipeline
`Pipeline(runner, cfg)`
| Método | Descripción |
|---|---|
| `process(frame_bgr) → dict` | Inferencia + todo el post-procesado |
| `process_result(res, w, h, infer_s=0) → dict` | Igual, a partir de una respuesta ya calculada (pruebas). Relee `cfg.deadzone`/`hysteresis` en cada llamada |

Resultado: `centroids`, `target`, `pos`, `held`, `deviation`, `flags`, `timing`, `infer_s`.

## Envío
| Clase | Descripción |
|---|---|
| `FlagSender(transport, heartbeat_s=0.2).send(flags) → bool` | Escribe si cambió o si venció el latido; devuelve si envió |
| `PrintTransport().write(flags)` | Imprime |
| `SerialTransport(port="/dev/ttyACM0", baud=115200)` | `write` envía 1 byte; espera 2 s al abrir (reset del Arduino); `close()` |
| `BridgeTransport(method="set_flags")` | `write` → `Bridge.call(method, int(flags))`. Requiere `arduino.app_utils` |

## Visualización
`draw_overlay(frame, out, cfg, fps=None) → img` dibuja ejes, rectángulo de zona muerta, centroides (verde), objetivo (rojo, o naranja si `HOLD`), flecha centro→objetivo, desviación en px y grados, byte en binario y FPS.

## Utilidades (solo en el notebook)
| Función | Cámara | Descripción |
|---|---|---|
| `benchmark(runner, n=10, size=(640, 480))` | No | Tiempos de features, DSP, clasificación, total y FPS máx. |
| `set_model_threshold(runner, model_info, min_score)` | No | Cambia el umbral interno del `.eim` |
| `simulate_mcu(flags) → str` | No | Qué haría `sketch.ino` con el byte |
| `steps_for_angle(deg, steps_per_rev=2048, gear_ratio=1.0) → int` | No | Grados → pasos |
| `snapshot(cfg)` | Sí | Un frame con overlay en matplotlib |
| `live(cfg, transport=None)` | Sí | Tiempo real; `q` para salir; envía `0` al terminar |
| `collect_dataset(cfg, label, out_dir="dataset")` | Sí | `s` guarda frames en `dataset/<label>/` |
| `calibrate_live(cfg)` | Sí | Sliders de deadzone/hysteresis/min_conf/smoothing; imprime los valores finales |
| `record_demo(cfg, path="demo_fomo.mp4", seconds=20, fps=15)` | Sí | Graba video con overlay |
