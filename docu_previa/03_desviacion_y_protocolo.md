# 03 · Desviación y protocolo binario

## Desviación respecto al centro
- **Eje central:** la mitad de la imagen `(W/2, H/2)`.
- **Punto de referencia:** el centroide FOMO (ya filtrado).

| Magnitud | Fórmula | Rango | Signo |
|---|---|---|---|
| `dx_px` | `cx − W/2` | ±W/2 | + derecha |
| `dy_px` | `H/2 − cy` | ±H/2 | + arriba (se invierte porque en la imagen y crece hacia abajo) |
| `dx_norm` | `dx_px / (W/2)` | [−1, 1] | |
| `dy_norm` | `dy_px / (H/2)` | [−1, 1] | |
| `dx_deg` | `atan(dx_norm · tan(HFOV/2))` | ±HFOV/2 | |
| `dy_deg` | `atan(dy_norm · tan(VFOV/2))` | ±VFOV/2 | |

Los grados usan el modelo de cámara *pinhole*. `HFOV`/`VFOV` (60°×45° por defecto) **hay que ajustarlos a la cámara real** para que sean exactos. Con un 28BYJ-48 (2048 pasos/vuelta): 1° ≈ 5.7 pasos.

## Estabilización antes de codificar
| Etapa | Parámetro | Efecto |
|---|---|---|
| Elección de objetivo | `target_strategy` | `closest_prev` (recomendado) sigue al mismo objeto; `confidence` toma el más seguro; `closest_center` el más cercano al centro |
| Suavizado EMA | `smoothing` (α) | `pos = α·nuevo + (1−α)·anterior`. 1.0 = sin suavizar; menor = más estable pero más lento |
| Hold | `lost_hold_frames` | Si FOMO pierde el objeto ≤ N frames se mantiene la última posición y se marca `HOLD` |

## Zona muerta e histéresis
Por cada eje hay un estado −1 / 0 / +1:
```
                  0 (centro)
          ◄────── entra a ±1 si |n| > deadzone
 estado ±1 ──────► vuelve a 0 solo si |n| < deadzone − hysteresis
```
Con `deadzone=0.10` y `hysteresis=0.03`: el objeto tiene que alejarse más del 10 % de media imagen para empezar a girar, y volver por debajo del 7 % para parar. Sin histéresis, un objeto justo en el borde haría oscilar al motor.

## Formato del byte
| bit | máscara | flag | significado |
|---|---|---|---|
| 0 | `0x01` | `DETECTED` | hay objetivo (real o estimado) |
| 1 | `0x02` | `LEFT` | objeto a la izquierda → girar a la izquierda |
| 2 | `0x04` | `RIGHT` | objeto a la derecha |
| 3 | `0x08` | `UP` | objeto arriba |
| 4 | `0x10` | `DOWN` | objeto abajo |
| 5 | `0x20` | `CENTERED` | dentro de la zona muerta en **ambos** ejes |
| 6 | `0x40` | `HOLD` | posición estimada (objeto perdido momentáneamente) |
| 7 | `0x80` | — | libre (modo, botón, etc.) |

Reglas:
- Sin objetivo → `0x00`.
- `LEFT` y `RIGHT` (o `UP` y `DOWN`) nunca están activos a la vez.
- `CENTERED` implica que no hay ningún bit de dirección.

Ejemplos:
| Byte | Texto | Acción en el sketch |
|---|---|---|
| `0b00000000` | NONE | motor detenido |
| `0b00100001` | DETECTED\|CENTERED | motor detenido ✓ |
| `0b00000011` | DETECTED\|LEFT | gira a la izquierda |
| `0b00001101` | DETECTED\|RIGHT\|UP | gira a la derecha (UP sin usar con un solo motor) |
| `0b01000011` | DETECTED\|LEFT\|HOLD | sigue girando a la izquierda con la última posición |

## Reglas de envío
| Regla | Valor | Dónde |
|---|---|---|
| Enviar solo si el byte cambia | — | `FlagSender.send` |
| Latido (reenviar aunque no cambie) | 200 ms | `FlagSender(heartbeat_s=0.2)` |
| Timeout en el MCU | 500 ms sin mensajes → parar | `TIMEOUT_MS` en los sketches |
| Al salir del loop en vivo | se envía `0x00` | `live()` del notebook |
| Si la cámara falla | se envía `0x00` | `loop()` de `main.py` |

## Transportes
| Clase | Medio | Formato |
|---|---|---|
| `PrintTransport` | consola | `→ MCU 0b00000011 (DETECTED\|LEFT)` |
| `SerialTransport(port, baud=115200)` | USB serial | 1 byte crudo; el sketch se queda con el último recibido |
| `BridgeTransport("set_flags")` | Bridge RPC del UNO Q | `Bridge.call("set_flags", int)` |
