# 02 · FOMO y el centroide

## Qué es FOMO
**FOMO** (*Faster Objects, More Objects*) es la arquitectura de detección de Edge Impulse pensada para microcontroladores y dispositivos pequeños. A diferencia de YOLO o SSD **no predice cajas**:

- Se entrena con el **centroide** de cada caja etiquetada.
- La salida es una **rejilla de probabilidades** de 1/8 de la resolución de entrada (96×96 → 12×12 celdas). Cada celda dice "aquí hay un objeto de clase X con probabilidad p".
- **Solo da la ubicación, no el tamaño.** No sirve para estimar distancia por tamaño aparente.
- Funciona mejor con objetos de tamaño parecido que no estén pegados entre sí.

## Qué devuelve el SDK
`runner.classify(features)` devuelve:
```python
{
  "result": {
    "bounding_boxes": [
      {"label": "audifono", "value": 0.87, "x": 40, "y": 24, "width": 8, "height": 8},
      ...
    ]
  },
  "timing": {"dsp": 6, "classification": 88, ...}   # ms
}
```
El SDK **fusiona** las celdas activas vecinas y las devuelve como `bounding_boxes`. Ese `width`/`height` es el tamaño de las celdas fusionadas, **no** el del objeto. Las coordenadas están en **píxeles de la entrada del modelo** (p. ej. 0–96), no del frame de la cámara.

## Cómo se obtiene el centroide
```
centroide_modelo = (x + width/2, y + height/2)
centroide_frame  = model_to_frame(centroide_modelo, resize_mode)
```
`model_to_frame` deshace el redimensionado que hizo el SDK según el *resize mode* configurado en Studio:

| Resize mode | Qué hace el SDK | Conversión inversa |
|---|---|---|
| `squash` | Estira el frame al tamaño del modelo (deforma) | `x · W/mw`, `y · H/mh` |
| `fit-shortest` | Recorta el centro con el aspecto del modelo y escala | escala + offset del recorte |
| `fit-longest` | Escala manteniendo aspecto y rellena con negro (letterbox) | resta el padding y divide por la escala |

El modelo de ejemplo usa `squash`.

## Umbral de confianza (dos niveles)
1. **Dentro del `.eim`**: `min_score` (0.5 por defecto). Lo que esté por debajo ni siquiera aparece en `bounding_boxes`. Se cambia con `set_model_threshold(runner, model_info, score)`.
2. **En Python**: `Config.min_confidence`. Filtra después.

Para detectar más, hay que bajar **ambos**.

## Resolución del modelo
El modelo de ejemplo es **1080×1080** → 1.1 M de features por frame. Medido en la laptop: ~35 ms preparando features + ~117 ms de clasificación ≈ **8 FPS**. En el UNO Q será bastante más lento. **Recomendación:** entrenar a **96×96 o 160×160**.

## Fuentes
- [Edge Impulse – Detect objects with centroids (FOMO)](https://docs.edgeimpulse.com/tutorials/end-to-end/object-detection-centroids.md)
- [Edge Impulse – Object detection](https://docs.edgeimpulse.com/docs/tutorials/end-to-end-tutorials/object-detection)
- [Foro EI – FOMO usa centroides, no cajas](https://forum.edgeimpulse.com/t/fomo-using-center-point-detection-and-not-bounding-boxes/14333)
