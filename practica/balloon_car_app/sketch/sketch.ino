// Sketch para el STM32 del Arduino UNO Q: carro con 2 servos de rotación continua.
//
// Python (Bridge RPC) decide; este sketch solo ejecuta:
//   drive(v, w)        v, w en [-100, 100]; v>0 adelante, w>0 gira a la derecha
//   raw_us(l, r)       pulso directo en µs para calibrar (0 = soltar ese servo)
//   set_cal(idx, val)  calibración por servo (ver CAL[])
//   set_running(on)    arrancar/parar desde Python (cuando el color viene por TARGET_COLOR)
//   get_color()        -1 si no se ha arrancado; si no 0 rojo, 1 verde, 2 azul
//
// Botón en BUTTON_PIN (a GND): pulsación corta = cambiar color (el LED parpadea
// 1/2/3 veces), pulsación larga = arrancar / PARAR (interruptor de emergencia).
// Sin mensajes durante TIMEOUT_MS → servos parados.
#include <Arduino_RouterBridge.h>

// 1 = librería Servo. Si no compila en el core Zephyr, poner 0 (pulsos a mano con micros()).
#define USE_SERVO_LIB 1
#if USE_SERVO_LIB
#include <Servo.h>
#endif

const int SERVO_PINS[2] = {9, 10};   // izquierdo, derecho
const int SIGN[2] = {+1, -1};        // el servo derecho va montado espejado
const int BUTTON_PIN = 2;
const unsigned long TIMEOUT_MS = 300;
const unsigned long LONG_PRESS_MS = 1000;

// Calibración (Python la sobrescribe con set_cal al arrancar). K en µs/unidad ×100.
enum { STOP_L, STOP_R, K_L_FWD, K_L_REV, K_R_FWD, K_R_REV, DB_L, DB_R, N_CAL };
volatile int CAL[N_CAL] = {1500, 1500, 200, 200, 200, 200, 20, 20};

volatile int cmdV = 0, cmdW = 0;
volatile unsigned long lastDriveMs = 0;
volatile int rawUs[2] = {0, 0};
volatile unsigned long lastRawMs = 0;
volatile bool running = false;
volatile int colorIdx = 0;

// --- Handlers RPC: SOLO guardan valores (nada de Bridge.call ni prints aquí) ---
void drive(int v, int w) { cmdV = v; cmdW = w; lastDriveMs = millis(); }
void raw_us(int l, int r) { rawUs[0] = l; rawUs[1] = r; lastRawMs = millis(); }
void set_cal(int idx, int value) { if (idx >= 0 && idx < N_CAL) CAL[idx] = value; }
void set_running(int on) { running = on != 0; }
int get_color() { return running ? colorIdx : -1; }

// --- Salida a los servos ---
#if USE_SERVO_LIB
Servo servos[2];
void writeServo(int i, int us) {
  if (us <= 0) { if (servos[i].attached()) servos[i].detach(); return; }
  if (!servos[i].attached()) servos[i].attach(SERVO_PINS[i]);
  servos[i].writeMicroseconds(us);
}
void servoTick() {}
#else
int pulseUs[2] = {0, 0};
unsigned long frameStartUs = 0;
void writeServo(int i, int us) { pulseUs[i] = us; }
void servoTick() {  // 50 Hz, no bloqueante
  unsigned long now = micros();
  if (now - frameStartUs >= 20000UL) {
    frameStartUs = now;
    for (int i = 0; i < 2; i++) digitalWrite(SERVO_PINS[i], pulseUs[i] > 0 ? HIGH : LOW);
  }
  unsigned long el = now - frameStartUs;
  for (int i = 0; i < 2; i++)
    if (el >= (unsigned long)pulseUs[i]) digitalWrite(SERVO_PINS[i], LOW);
}
#endif

// val en [-100, 100] → µs con la calibración del servo i
int usFor(int i, int val) {
  int stop = CAL[STOP_L + i];
  if (val == 0) return stop;
  int k = val > 0 ? CAL[i == 0 ? K_L_FWD : K_R_FWD] : CAL[i == 0 ? K_L_REV : K_R_REV];
  int db = CAL[DB_L + i];
  int off = (long)val * k / 100 + (val > 0 ? db : -db);
  return constrain(stop + SIGN[i] * off, 1000, 2000);
}

// --- Botón y LED ---
bool btnPrev = false;
unsigned long btnDownMs = 0;

void buttonTick() {
  bool down = digitalRead(BUTTON_PIN) == LOW;
  unsigned long now = millis();
  if (down && !btnPrev) btnDownMs = now;
  if (!down && btnPrev) {
    unsigned long held = now - btnDownMs;
    if (held >= LONG_PRESS_MS) running = !running;
    else if (held > 30 && !running) colorIdx = (colorIdx + 1) % 3;
  }
  btnPrev = down;
}

void ledTick() {
  if (running) { digitalWrite(LED_BUILTIN, HIGH); return; }
  // colorIdx+1 parpadeos y una pausa, en un ciclo de 2 s
  unsigned long t = millis() % 2000;
  int blink = t / 250;
  bool on = blink < 2 * (colorIdx + 1) && (blink % 2 == 0);
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

void setup() {
  for (int i = 0; i < 2; i++) pinMode(SERVO_PINS[i], OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Bridge.begin();
  // provide_safe: el handler corre en el loop principal (thread-safe)
  Bridge.provide_safe("drive", drive);
  Bridge.provide_safe("raw_us", raw_us);
  Bridge.provide_safe("set_cal", set_cal);
  Bridge.provide_safe("set_running", set_running);
  Bridge.provide_safe("get_color", get_color);
}

void loop() {
  buttonTick();
  ledTick();
  unsigned long now = millis();

  if (now - lastRawMs < TIMEOUT_MS) {            // modo calibración
    writeServo(0, rawUs[0]);
    writeServo(1, rawUs[1]);
  } else if (running && now - lastDriveMs < TIMEOUT_MS) {
    int v = cmdV, w = cmdW;
    int l = constrain(v + w, -100, 100);
    int r = constrain(v - w, -100, 100);
    writeServo(0, usFor(0, l));
    writeServo(1, usFor(1, r));
  } else {
    writeServo(0, 0);                           // parado: sin pulso, el servo no se mueve
    writeServo(1, 0);                           // aunque STOP_x esté mal calibrado
  }
  servoTick();
}
