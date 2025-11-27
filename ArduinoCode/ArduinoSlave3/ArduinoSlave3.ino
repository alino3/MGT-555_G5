#include <Wire.h>
#include <Servo.h>

#define STEP_PIN_3 2
#define DIR_PIN_3 3
#define ENABLE_PIN_3 4
#define SERVO_1_PIN 5
#define SERVO_2_PIN 6

Servo servo1, servo2;
bool isMovingStepper = false;
unsigned long stepperStartTime = 0;
long remainingSteps = 0;
byte stepperDirection = 0;

void setup() {
  Serial.begin(115200);
  
  // Stepper setup
  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(ENABLE_PIN_3, OUTPUT);
  digitalWrite(ENABLE_PIN_3, LOW);
  
  // Servo setup
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  servo1.write(90);
  servo2.write(90);
  
  // I2C setup
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
  
  Serial.println("Arduino 3 Ready - Improved Version");
}

void receiveEvent(int bytes) {
  Serial.print("I2C: "); Serial.print(bytes); Serial.println(" bytes");
  
  if (bytes >= 1) {
    byte first = Wire.read();
    
    if (first == 0xAA && bytes == 3) {
      // Servo command - process immediately
      byte s1 = Wire.read();
      byte s2 = Wire.read();
      Serial.print("Servos: "); Serial.print(s1); Serial.print(","); Serial.println(s2);
      setServos(s1, s2);
      
    } else if (bytes == 5) {
      // Stepper command - store for later processing
      long steps = first;
      for (int i = 0; i < 3; i++) {
        steps = (steps << 8) | Wire.read();
      }
      byte dir = Wire.read();
      
      // If already moving, stop current movement
      if (isMovingStepper) {
        Serial.println("Interrupting current movement");
        isMovingStepper = false;
      }
      
      // Start new movement
      startStepperMovement(steps, dir);
    }
  }
}

void startStepperMovement(long steps, byte direction) {
  remainingSteps = steps;
  stepperDirection = direction;
  isMovingStepper = true;
  stepperStartTime = millis();
  
  digitalWrite(DIR_PIN_3, direction);
  delay(10);
  
  Serial.print("Starting stepper: "); Serial.print(steps); Serial.println(" steps");
}

void setServos(byte angle1, byte angle2) {
  // Servo commands can be processed even while stepper is moving
  servo1.write(angle1);
  servo2.write(angle2);
  Serial.print("Servos set to: "); Serial.print(angle1); Serial.print(","); Serial.println(angle2);
}

void loop() {
  // Handle stepper movement in loop to keep I2C responsive
  if (isMovingStepper && remainingSteps > 0) {
    digitalWrite(STEP_PIN_3, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN_3, LOW);
    delayMicroseconds(800);
    remainingSteps--;
    
    if (remainingSteps % 100 == 0) {
      Serial.print("Stepper progress: "); Serial.println(remainingSteps);
    }
  } else if (isMovingStepper && remainingSteps == 0) {
    isMovingStepper = false;
    Serial.println("Stepper movement completed");
  }
}