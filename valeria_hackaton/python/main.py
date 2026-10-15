from arduino.app_utils import App
from test_motores import *

# Ejecutamos la secuencia de prueba
test_todas_las_funciones()

# Mantenemos el hilo principal vivo por si el entorno de Arduino lo requiere
App.run()