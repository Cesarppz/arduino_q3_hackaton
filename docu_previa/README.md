# Documentación previa: FOMO Tracker (hackatón Arduino UNO Q)

Sistema que detecta un objeto con un modelo **FOMO** de Edge Impulse, calcula cuánto se desvía del **centro de la cámara** y envía un **byte de flags** a un microcontrolador para mover un motor que centre el objeto.

## Índice
| Documento | Contenido |
|---|---|
| [01_arquitectura.md](01_arquitectura.md) | Visión general, componentes, flujo de datos y estructura de archivos |
| [02_fomo.md](02_fomo.md) | Qué es FOMO, qué devuelve y cómo se obtiene el centroide |
| [03_desviacion_y_protocolo.md](03_desviacion_y_protocolo.md) | Cálculo de la desviación, zona muerta, histéresis y formato del byte |
| [04_uno_q_bridge.md](04_uno_q_bridge.md) | Arduino UNO Q: comunicación Python ↔ MCU con Bridge y la app de App Lab |
| [05_referencia_api.md](05_referencia_api.md) | Referencia de `fomo_core.py` (clases y funciones) |
| [06_guia_hackaton.md](06_guia_hackaton.md) | Paso a paso para el día del hackatón, utilidades y troubleshooting |

## Inicio rápido
```bash
# 1. Abrir practica/fomo_desviacion.ipynb en PyCharm / VS Code con el kernel del .venv
#    → poner RUN_CAMERA = True para usar la cámara

# 2. Probar la app del UNO Q en la laptop
cd practica/uno_q_app/python
EI_MODEL=../../cesar727-project-1-linux-x86_64-v2-fomo.eim ../../../.venv/bin/python main.py
```

## Estado actual
| Parte | Estado |
|---|---|
| Carga del `.eim` de ejemplo (x86_64, FOMO, etiqueta `audifono`, 1080×1080) | ✅ probado |
| Pipeline centroide → desviación → flags (datos sintéticos) | ✅ probado con asserts |
| `main.py` en modo laptop y con Bridge simulado | ✅ probado |
| Sintaxis de los sketches (g++ con cabeceras simuladas) | ✅ probado |
| Cámara en vivo, motor real, placa UNO Q | ⏳ pendiente (requiere hardware) |
| Modelo entrenado funcional | ⏳ el `.eim` de ejemplo no detecta nada |
