// Sketch para el STM32 del Arduino UNO Q: carro con 2 motores DC (dirección + PWM).
//
// La configuración de los motores es la de la rama `valeria` (move.cpp / move.h):
//   izquierdo: dir 13, PWM 11 · derecho: dir 12, PWM 3 · velocidad en [-100, 100].
//
// Encima de eso, Python (Bridge RPC) controla el carro con:
//   drive(v, w)         v, w en [-100, 100]; v>0 adelante, w>0 gira a la derecha
//   raw_pct(l, r)       velocidad directa por motor, sin compensación (calibración)
//   set_cal(idx, val)   compensación por motor (ver CAL[])
//   set_running(on)     arrancar / parar
//   get_running()       1 si está en marcha
// Siguen disponibles las funciones de move.cpp (Go, MotorIzquierdo, ...) para
// los scripts de prueba de la rama `valeria`.
//
// Botón en BUTTON_PIN (a GND): cada pulsación arranca / PARA (emergencia).
// Sin mensajes de drive/raw_pct durante TIMEOUT_MS → motores parados.
#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include "move.h"

const int BUTTON_PIN = 2;
const unsigned long TIMEOUT_MS = 300;

// Compensación de cada motor (Python la sobrescribe con set_cal al arrancar):
//   GAIN_x: % de la velocidad pedida (iguala motores con distinto impulso)
//   MIN_x : % mínimo para que la rueda empiece a girar (zona muerta del motor DC)
enum { GAIN_IZQ, GAIN_DER, MIN_IZQ, MIN_DER, N_CAL };
volatile int CAL[N_CAL] = {100, 100, 20, 20};

volatile int cmdV = 0, cmdW = 0;
volatile int rawL = 0, rawR = 0;
volatile unsigned long lastDriveMs = 0, lastRawMs = 0;
volatile bool running = false;
bool moving = false;   // hubo un comando vivo en la vuelta anterior

// --- Handlers RPC: SOLO guardan valores ---
void drive(int v, int w) { cmdV = v; cmdW = w; lastDriveMs = millis(); }
void raw_pct(int l, int r) { rawL = l; rawR = r; lastRawMs = millis(); }
void set_cal(int idx, int value) { if (idx >= 0 && idx < N_CAL) CAL[idx] = value; }
void set_running(int on) { running = on != 0; }
int get_running() { return running ? 1 : 0; }

// [-100, 100] → velocidad compensada: salta la zona muerta y aplica la ganancia
int compensate(int val, int gain, int minPct) {
  if (val == 0) return 0;
  int mag = minPct + (long)abs(val) * (100 - minPct) / 100;
  mag = constrain((long)mag * gain / 100, 0, 100);
  return val > 0 ? mag : -mag;
}

bool btnPrev = false;
unsigned long btnDownMs = 0;

void buttonTick() {
  bool down = digitalRead(BUTTON_PIN) == LOW;
  unsigned long now = millis();
  if (down && !btnPrev) btnDownMs = now;
  if (!down && btnPrev && now - btnDownMs > 30) running = !running;
  btnPrev = down;
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ConfigurarMotores();

  Bridge.begin();
  Bridge.provide_safe("drive", drive);
  Bridge.provide_safe("raw_pct", raw_pct);
  Bridge.provide_safe("set_cal", set_cal);
  Bridge.provide_safe("set_running", set_running);
  Bridge.provide_safe("get_running", get_running);
  // Funciones de la rama valeria (pruebas manuales)
  Bridge.provide_safe("MotorIzquierdo", MotorIzquierdo);
  Bridge.provide_safe("MotorDerecho", MotorDerecho);
  Bridge.provide_safe("Go", Go);
  Bridge.provide_safe("PararMotores", PararMotores);
  Bridge.provide_safe("GoMotores", GoMotores);
}

void loop() {
  Bridge.update();
  buttonTick();
  unsigned long now = millis();

  if (now - lastRawMs < TIMEOUT_MS) {                  // calibración
    MotorIzquierdo(constrain((int)rawL, -100, 100));
    MotorDerecho(constrain((int)rawR, -100, 100));
    moving = true;
  } else if (running && now - lastDriveMs < TIMEOUT_MS) {
    int v = cmdV, w = cmdW;
    int l = constrain(v + w, -100, 100);
    int r = constrain(v - w, -100, 100);
    MotorIzquierdo(compensate(l, CAL[GAIN_IZQ], CAL[MIN_IZQ]));
    MotorDerecho(compensate(r, CAL[GAIN_DER], CAL[MIN_DER]));
    moving = true;
  } else if (moving) {
    // Se para una sola vez al perder los comandos (o con el botón), así las
    // llamadas directas a Go/MotorIzquierdo de las pruebas siguen funcionando.
    PararMotores();
    moving = false;
  }
  delay(2);
}
