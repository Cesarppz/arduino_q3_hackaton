// Sketch para el STM32 del Arduino UNO Q.
// Recibe un byte de flags desde Python (Bridge RPC "set_flags") y mueve un
// stepper 28BYJ-48 (ULN2003) hacia el objeto hasta que quede centrado.
//
// Flags (mismo orden que fomo_core.py):
//   bit0 DETECTED  bit1 LEFT  bit2 RIGHT  bit3 UP  bit4 DOWN  bit5 CENTERED  bit6 HOLD
#include <Arduino_RouterBridge.h>

const uint8_t FLAG_DETECTED = 1 << 0;
const uint8_t FLAG_LEFT     = 1 << 1;
const uint8_t FLAG_RIGHT    = 1 << 2;
const uint8_t FLAG_UP       = 1 << 3;
const uint8_t FLAG_DOWN     = 1 << 4;
const uint8_t FLAG_CENTERED = 1 << 5;
const uint8_t FLAG_HOLD     = 1 << 6;

// Mismos pines que practica_stepper.ino: Stepper(2048, 8, 10, 9, 11)
const int MOTOR_PINS[4] = {8, 10, 9, 11};
const int STEPS_PER_REV = 2048;
const int RPM = 10;
const unsigned long STEP_INTERVAL_US = 60UL * 1000UL * 1000UL / STEPS_PER_REV / RPM;
const unsigned long TIMEOUT_MS = 500;  // sin mensajes de Python → parar

// Secuencia de 4 pasos de la librería Stepper (4 cables), implementada a mano
// para que sea no bloqueante y no dependa de la librería en el core Zephyr.
const uint8_t SEQ[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1},
};

volatile uint8_t flags = 0;
volatile unsigned long lastMsgMs = 0;
int seqIndex = 0;
unsigned long lastStepUs = 0;

// Handler RPC: SOLO guarda el valor. No llamar Bridge.call ni imprimir aquí
// (la documentación advierte de posibles deadlocks).
void set_flags(int value) {
  flags = (uint8_t)value;
  lastMsgMs = millis();
}

void writeCoils(int idx) {
  for (int i = 0; i < 4; i++) digitalWrite(MOTOR_PINS[i], SEQ[idx][i]);
}

void releaseCoils() {
  for (int i = 0; i < 4; i++) digitalWrite(MOTOR_PINS[i], LOW);
}

void stepOnce(int dir) {
  seqIndex = (seqIndex + dir + 4) % 4;
  writeCoils(seqIndex);
}

void setup() {
  for (int i = 0; i < 4; i++) pinMode(MOTOR_PINS[i], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  releaseCoils();

  Bridge.begin();
  // provide_safe: el handler se ejecuta en el loop principal (thread-safe)
  Bridge.provide_safe("set_flags", set_flags);
}

void loop() {
  uint8_t f = flags;
  bool alive = (millis() - lastMsgMs) < TIMEOUT_MS;

  digitalWrite(LED_BUILTIN, (alive && (f & FLAG_DETECTED)) ? HIGH : LOW);

  int dir = 0;
  if (alive && (f & FLAG_DETECTED) && !(f & FLAG_CENTERED)) {
    if (f & FLAG_LEFT)  dir = -1;   // invierte el signo si el motor gira al revés
    if (f & FLAG_RIGHT) dir = +1;
  }

  if (dir == 0) {
    releaseCoils();            // evita que el motor se caliente en reposo
    return;
  }

  unsigned long now = micros();
  if (now - lastStepUs >= STEP_INTERVAL_US) {
    lastStepUs = now;
    stepOnce(dir);
  }

  // FLAG_UP/FLAG_DOWN están disponibles para un segundo motor (tilt) con la misma lógica.
}
