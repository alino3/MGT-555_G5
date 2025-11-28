#include <Servo.h>

Servo s;

// --- Connexions ---
const int SERVO_PIN = 10;   // fil signal du servo

// --- Calibration du servo DS3218 sous 5 V ---
const int SERVO_MIN_US = 600;     // ≈ 0°
const int SERVO_MAX_US = 2400;    // ≈ 180°

// --- Plage d'utilisation ---
const int ANG_MIN = 10;       // bas
const int ANG_MAX = 136;     // haut

// --- Vitesses ---
const unsigned long GO_INTERVAL_MS   = 15;  // aller lent
const unsigned long BACK_INTERVAL_MS = 5;   // retour rapide
const int STEP_DEG = 1;

// --- Machine d'états ---
enum State { IDLE, MOVING_FWD, MOVING_BACK };
State state = IDLE;

// Variables
int currentAngle = ANG_MAX;      // <-- NOUVEAU : on démarre en haut
int targetAngle  = ANG_MAX;
int direction    = -1;
unsigned long lastStepTime = 0;
unsigned long stepInterval = GO_INTERVAL_MS;

// Convertit angle en microsecondes
int angleToMicros(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
}

void writeAngle(int deg) {
  s.writeMicroseconds(angleToMicros(deg));
}

// Prépare un mouvement
void startMove(int toAngle, unsigned long intervalMs) {
  targetAngle  = constrain(toAngle, ANG_MIN, ANG_MAX);
  direction    = (targetAngle > currentAngle) ? +1 : -1;
  stepInterval = intervalMs;
  lastStepTime = 0;
}

// Avance d'un pas si c'est l'heure
void doStepIfDue() {
  if (millis() - lastStepTime < stepInterval) return;
  lastStepTime = millis();

  int next = currentAngle + direction * STEP_DEG;

  if ((direction > 0 && next > targetAngle) ||
      (direction < 0 && next < targetAngle)) {
    next = targetAngle;
  }

  currentAngle = next;
  writeAngle(currentAngle);
}

void setup() {
  Serial.begin(115200);
  s.attach(SERVO_PIN);

  // Démarre à ANG_MAX (nouveau "départ" inversé)
  writeAngle(ANG_MAX);
  currentAngle = ANG_MAX;

  Serial.println("Envoyer 'E' pour lancer l'aller-retour (sens inverse).");
}

void loop() {

  switch(state) {

    case IDLE:
      // Déclenchement par 'E'
      if (Serial.available() && Serial.read() == 'E') {
        startMove(ANG_MIN, GO_INTERVAL_MS);   // aller lent : 135° → 0°
        state = MOVING_FWD;
      }
      break;

    case MOVING_FWD:
      if (currentAngle != targetAngle) {
        doStepIfDue();
      } else {
        startMove(ANG_MAX, BACK_INTERVAL_MS);  // retour rapide : 0° → 135°
        state = MOVING_BACK;
      }
      break;

    case MOVING_BACK:
      if (currentAngle != targetAngle) {
        doStepIfDue();
      } else {
        state = IDLE;   // prêt pour un nouveau cycle
      }
      break;
  }
}
