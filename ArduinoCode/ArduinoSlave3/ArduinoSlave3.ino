#include <Wire.h>
#include <Servo.h>

#define STEP_PIN_2 2
#define DIR_PIN_2 3
#define ENABLE_PIN_2 4
#define SERVO_PHI_PIN 5
#define SERVO_GRIPPER_PIN 6

Servo servoPhi;
Servo servoGripper;

// VOLATILE is required for variables shared with Interrupts
volatile long receivedSteps = 0;
volatile byte receivedDirection = 0;
volatile bool newData = false;

volatile byte receivedServoPhi = 0xFF;
volatile byte receivedServoGripper = 0xFF;
volatile bool newServoData = false;

void setup() {
  // Init Pins
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW);
  
  // Init Servos
  servoPhi.attach(SERVO_PHI_PIN);
  servoGripper.attach(SERVO_GRIPPER_PIN);
  servoPhi.write(90);
  servoGripper.write(90);
  
  // Start I2C
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(9600);
  Serial.println("Arduino 3 Ready (Fixed Protocol)");
}

void loop() {
  // Check for Servo Data
  if (newServoData) {
    // Disable interrupts briefly to read shared variables safely
    noInterrupts();
    byte phi = receivedServoPhi;
    byte grip = receivedServoGripper;
    newServoData = false;
    interrupts();
    
    if (phi != 0xFF) {
      Serial.print("Servo Phi: "); Serial.println(phi);
      servoPhi.write(phi);
    }
    if (grip != 0xFF) {
      Serial.print("Servo Grip: "); Serial.println(grip);
      servoGripper.write(grip);
    }
  }

  // Check for Stepper Data
  if (newData) {
    noInterrupts();
    long steps = receivedSteps;
    byte dir = receivedDirection;
    newData = false;
    interrupts();
    
    Serial.print("Moving Z: "); Serial.println(steps);
    
    // Move Motor (Blocking)
    digitalWrite(DIR_PIN_2, dir);
    for (long i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN_2, HIGH);
      delayMicroseconds(800);
      digitalWrite(STEP_PIN_2, LOW);
      delayMicroseconds(800);
    }
    Serial.println("Z Move Done");
  }
}

// INTERRUPT SERVICE ROUTINE - NO SERIAL PRINTS HERE!
void receiveEvent(int bytes) {
  if (bytes < 1) return;
  
  byte firstByte = Wire.read();
  
  // CASE 1: Combined Command (0xBB header)
  if (firstByte == 0xBB && bytes >= 6) {
    // Read Steps (4 bytes)
    receivedSteps = 0;
    for (int i = 0; i < 4; i++) {
      receivedSteps = (receivedSteps << 8) | Wire.read();
    }
    receivedDirection = Wire.read();
    newData = true;
    
    // Check if Servo data is attached (Separator 0xAA + 2 bytes)
    if (Wire.available() >= 3) {
       byte sep = Wire.read(); // Read the 0xAA separator
       if (sep == 0xAA) {
          receivedServoPhi = Wire.read();
          receivedServoGripper = Wire.read();
          newServoData = true;
       }
    }
  }
  // CASE 2: Servo Only Command (0xAA header)
  else if (firstByte == 0xAA && bytes >= 3) {
    receivedServoPhi = Wire.read();
    receivedServoGripper = Wire.read();
    newServoData = true;
  }
  
  // Clear any garbage remaining
  while(Wire.available()) Wire.read();
}