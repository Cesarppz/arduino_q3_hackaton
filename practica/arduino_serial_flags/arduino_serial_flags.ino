// Versión para Arduino clásico (UNO R3/R4) conectado por USB a la laptop.
// Recibe el mismo byte de flags que la app del UNO Q, pero por Serial
// (SerialTransport en fomo_core.py). Sirve para probar el motor sin el UNO Q.
const uint8_t FLAG_DETECTED = 1 << 0;
const uint8_t FLAG_LEFT     = 1 << 1;
const uint8_t FLAG_RIGHT    = 1 << 2;
const uint8_t FLAG_CENTERED = 1 << 5;

const int MOTOR_PINS[4] = {8, 10, 9, 11};
const int STEPS_PER_REV = 2048;
const int RPM = 10;
const unsigned long STEP_INTERVAL_US = 60UL * 1000UL * 1000UL / STEPS_PER_REV / RPM;
const unsigned long TIMEOUT_MS = 500;

const uint8_t SEQ[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1},
};

uint8_t flags = 0;
unsigned long lastMsgMs = 0;
int seqIndex = 0;
unsigned long lastStepUs = 0;

void writeCoils(int idx) {
  for (int i = 0; i < 4; i++) digitalWrite(MOTOR_PINS[i], SEQ[idx][i]);
}

void releaseCoils() {
  for (int i = 0; i < 4; i++) digitalWrite(MOTOR_PINS[i], LOW);
}

void setup() {
  for (int i = 0; i < 4; i++) pinMode(MOTOR_PINS[i], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  releaseCoils();
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {   // nos quedamos con el byte más reciente
    flags = Serial.read();
    lastMsgMs = millis();
  }
  bool alive = (millis() - lastMsgMs) < TIMEOUT_MS;
  digitalWrite(LED_BUILTIN, (alive && (flags & FLAG_DETECTED)) ? HIGH : LOW);

  int dir = 0;
  if (alive && (flags & FLAG_DETECTED) && !(flags & FLAG_CENTERED)) {
    if (flags & FLAG_LEFT)  dir = -1;
    if (flags & FLAG_RIGHT) dir = +1;
  }
  if (dir == 0) {
    releaseCoils();
    return;
  }
  unsigned long now = micros();
  if (now - lastStepUs >= STEP_INTERVAL_US) {
    lastStepUs = now;
    seqIndex = (seqIndex + dir + 4) % 4;
    writeCoils(seqIndex);
  }
}
