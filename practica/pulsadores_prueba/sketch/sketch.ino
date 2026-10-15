#include <Arduino_RouterBridge.h>
#include "move.h"

RobotControl robot;

void setup() {
  Bridge.begin();
  robot.inicializar();
  Serial.println("--- PRUEBA DE PULSADORES Y LEDS INICIADA ---");
}

void loop() {
  // Comprueba continuamente los pulsadores y enciende sus LEDs correspondientes
  robot.verificarPulsadores();
}