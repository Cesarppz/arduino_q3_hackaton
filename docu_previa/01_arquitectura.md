# 01 · Arquitectura

## Flujo de datos
```
┌──────────────┐    frame BGR    ┌────────────────────────────── MPU (Linux) ─────────────────────────────┐
│  Cámara USB  │ ──────────────► │ run_inference ─► get_centroids ─► pick_target ─► CentroidFilter         │
└──────────────┘                 │   (.eim FOMO)     (centro de        (1 objetivo)    (EMA + hold)         │
                                 │                    cada caja)                            │              │
                                 │                                          compute_deviation ◄┘              │
                                 │                                                  │                         │
                                 │                                  DirectionEncoder ─► FlagSender            │
                                 └──────────────────────────────────────────────────────────┬────────────────┘
                                                                                            │ 1 byte
                                          Bridge RPC (UNO Q)  /  Serial USB (Arduino clásico) │
                                                                                            ▼
                                 ┌──────────────────────── MCU (STM32 / AVR) ───────────────────────────────┐
                                 │ set_flags(byte) → guarda el byte → loop(): paso del stepper LEFT/RIGHT     │
                                 │ timeout 500 ms sin mensajes → motor detenido                              │
                                 └───────────────────────────────────────────────────────────────────────────┘
```

## Componentes
| Componente | Dónde corre | Responsabilidad |
|---|---|---|
| Modelo `.eim` | MPU (Linux) | Binario de Edge Impulse; el SDK `edge_impulse_linux` lo lanza como proceso y se comunica por socket |
| `fomo_core.py` | MPU | Toda la lógica: inferencia, centroides, filtro, desviación, codificación y transportes |
| `main.py` | MPU | Loop de la app: cámara → `Pipeline` → `FlagSender` |
| `fomo_desviacion.ipynb` | Laptop | Mismo código explicado paso a paso, pruebas y utilidades |
| `sketch.ino` | MCU del UNO Q (STM32U585, Zephyr) | Recibe el byte por Bridge y mueve el stepper |
| `arduino_serial_flags.ino` | Arduino clásico (UNO R3/R4) | Igual, pero recibe por Serial; sirve para probar sin el UNO Q |

## Estructura de archivos
```
practica/
├── fomo_desviacion.ipynb               notebook principal (laptop)
├── cesar727-project-1-linux-x86_64-v2-fomo.eim   modelo de ejemplo (no entrenado)
├── arduino_serial_flags/
│   └── arduino_serial_flags.ino        receptor por Serial (Arduino clásico)
├── practica_stepper/
│   └── practica_stepper.ino            práctica original del stepper (pines de referencia)
└── uno_q_app/                          app para Arduino App Lab
    ├── app.yaml
    ├── README.md
    ├── python/
    │   ├── main.py
    │   ├── fomo_core.py                ← fuente única de la lógica
    │   ├── requirements.txt
    │   └── model.eim                   ← poner aquí el build Linux AARCH64
    └── sketch/
        └── sketch.ino
docu_previa/                            esta documentación
```

## Decisiones de diseño
| Decisión | Motivo |
|---|---|
| Inferencia en Python con `.eim` propio (no el brick `video_object_detection`) | El mismo código corre en la laptop y en el UNO Q; control total del post-procesado |
| Salida = byte de flags de dirección | El MCU solo necesita decidir girar izquierda/derecha/parar; mínimo ancho de banda y lógica trivial en C++ |
| Envío solo al cambiar + latido de 200 ms | No saturar el bus y permitir un *timeout* de seguridad en el MCU |
| `fomo_core.py` como fuente única | El notebook muestra sus secciones `# %% [...]`; `main.py` lo importa. Evita que laptop y placa se comporten distinto |
| Stepper sin librería, no bloqueante | La librería `Stepper` bloquea y no está garantizada en el core Zephyr del UNO Q |

> **Sincronía notebook ↔ app:** el notebook se generó a partir de `fomo_core.py`. Si editas el notebook, copia el cambio al archivo (o al revés).
