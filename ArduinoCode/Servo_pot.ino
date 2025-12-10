#include <Servo.h>

Servo s;

// --- Connexions ---
const int SERVO_PIN = 9;

// --- Calibration PWM ---
const int SERVO_MIN_US = 600;
const int SERVO_MAX_US = 2400;

// Servo ≈ 225°
const int SERVO_MAX_DEG = 225;

// --- Butées mécaniques (à ajuster) ---
int LEFT_LIMIT  = 5;      // CLOSE
int RIGHT_LIMIT = 220;    // OPEN

// --- Vitesse ---
const unsigned long MOVE_INTERVAL_MS = 5;
const int STEP_DEG = 1;

// --- Etats ---
enum State { IDLE, MOVING };
State state = IDLE;

int currentAngle = LEFT_LIMIT;   // démarrage = CLOSE
int targetAngle  = LEFT_LIMIT;
int direction    = +1;
unsigned long lastStepTime = 0;

// Conversion angle → µs
int angleToMicros(int deg) {
  deg = constrain(deg, 0, SERVO_MAX_DEG);
  return map(deg, 0, SERVO_MAX_DEG, SERVO_MIN_US, SERVO_MAX_US);
}

void writeAngle(int deg) {
  s.writeMicroseconds(angleToMicros(deg));
}

void startMove(int toAngle) {
  targetAngle = constrain(toAngle, LEFT_LIMIT, RIGHT_LIMIT);
  direction   = (targetAngle > currentAngle) ? +1 : -1;
  lastStepTime = 0;
  state = MOVING;
}

void doStepIfDue() {
  if (millis() - lastStepTime < MOVE_INTERVAL_MS) return;
  lastStepTime = millis();

  int next = currentAngle + direction * STEP_DEG;

  if ((direction > 0 && next > targetAngle) ||
      (direction < 0 && next < targetAngle)) {
    next = targetAngle;
  }

  currentAngle = next;
  writeAngle(currentAngle);

  if (currentAngle == targetAngle) {
    state = IDLE;
  }
}

void handleSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (state != IDLE) return;

  if (c == 'O') {
    // OUVERTURE = aller à OPEN = RIGHT_LIMIT
    if (currentAngle != RIGHT_LIMIT) {
      startMove(RIGHT_LIMIT);
    }
  } 
  else if (c == 'C') {
    // FERMETURE = aller à CLOSE = LEFT_LIMIT
    if (currentAngle != LEFT_LIMIT) {
      startMove(LEFT_LIMIT);
    }
  }
}

void setup() {
  Serial.begin(115200);
  s.attach(SERVO_PIN);

  // démarrage EN CLOSE
  currentAngle = LEFT_LIMIT;
  writeAngle(currentAngle);

  Serial.println("Commandes :");
  Serial.println("  O -> ouvrir");
  Serial.println("  C -> fermer");
}

void loop() {
  handleSerial();
  if (state == MOVING) {
    doStepIfDue();
  }
}
