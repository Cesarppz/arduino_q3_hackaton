#include "pulsadores.h"

// Definición de pines según la documentación oficial del robot[cite: 5]
const int PIN_PULSADOR_1 = 10; // Pulsador 1 (P1) -> LED rojo[cite: 5]
const int PIN_PULSADOR_2 = 2;  // Pulsador 2 (P2) -> LED verde[cite: 5]
const int PIN_PULSADOR_3 = 0;  // Pulsador 3 (P3) -> Ambos LEDs[cite: 5]

const int PIN_LED_ROJO = 7;    // LED rojo[cite: 5]
const int PIN_LED_VERDE = 6;   // LED verde[cite: 5]

// Variable interna para recordar el color seleccionado actualmente
int colorSeleccionado = 0;

void RobotControl::inicializar() {
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  
  pinMode(PIN_PULSADOR_1, INPUT_PULLUP);
  pinMode(PIN_PULSADOR_2, INPUT_PULLUP);
  pinMode(PIN_PULSADOR_3, INPUT_PULLUP);
  
  Serial.begin(9600);
}

int RobotControl::verificarPulsadores() {
  int estadopuls1 = digitalRead(PIN_PULSADOR_1);
  int estadopuls2 = digitalRead(PIN_PULSADOR_2);
  int estadopuls3 = digitalRead(PIN_PULSADOR_3);
  
  // Si pulsamos el botón 1 (Pin 10): Selecciona y mantiene el LED rojo
  if (estadopuls1 == LOW) {
    encender(PIN_LED_ROJO);
    apagar(PIN_LED_VERDE);
    colorSeleccionado = 1; // Rojo
    delay(200); // Antirebote
  }
  // Si pulsamos el botón 2 (Pin 2): Selecciona y mantiene el LED verde
  else if (estadopuls2 == LOW) {
    apagar(PIN_LED_ROJO);
    encender(PIN_LED_VERDE);
    colorSeleccionado = 2; // Verde
    delay(200); // Antirebote
  }
  // Si pulsamos el botón 3 (Pin 0): Selecciona y mantiene ambos LEDs
  else if (estadopuls3 == LOW) {
    encender(PIN_LED_ROJO);
    encender(PIN_LED_VERDE);
    colorSeleccionado = 3; // Ambos / Azul
    delay(200); // Antirebote
  }
  
  // Devolvemos el color actual para que el main sepa qué se eligió
  return colorSeleccionado;
}

void RobotControl::encender(int led) {
  digitalWrite(led, HIGH);
  Serial.println("Led encendido");
}

void RobotControl::apagar(int led) {
  digitalWrite(led, LOW);
  Serial.println("Led apagado");
}