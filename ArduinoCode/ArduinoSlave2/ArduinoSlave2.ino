#include <Wire.h>

// Motor 2 pins (THIS ARDUINO CONTROLS MOTOR 2)
#define STEP_PIN_2 2    // → STEP on driver (PUL+)
#define DIR_PIN_2 3     // → DIR on driver (DIR+) 
#define ENABLE_PIN_2 4  // → ENABLE on driver

// Motor data
long receivedSteps = 0;     // MICROSTEPS received via I2C
byte receivedDirection = 0;
bool newData = false;

void setup() {
  // Initialize motor pins
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  
  // Enable motor (LOW = enabled)
  digitalWrite(ENABLE_PIN_2, LOW);
  digitalWrite(STEP_PIN_2, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  
  // Initialize I2C as slave with address 8
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(115200);
  Serial.println("Arduino 2 Ready - I2C Slave (Motor 2)");
}

void loop() {
  if (newData) {
    Serial.println("=== EXECUTING RECEIVED COMMAND ===");
    Serial.print("Motor 2: "); Serial.print(receivedSteps); Serial.print(" microsteps, dir: "); Serial.println(receivedDirection);
    
    moveMotor2();
    newData = false;
    
    Serial.println("=== COMMAND COMPLETED ===");
  }
}

void receiveEvent(int bytes) {
  if (bytes == 5) {
    receivedSteps = ... // reading logic
    receivedDirection = Wire.read();
    newData = true; 
  }
}

void moveMotor2() {
  // Set direction
  digitalWrite(DIR_PIN_2, receivedDirection);
  delay(10);
  
  // Generate steps (MICROSTEPS - each pulse is one microstep)
  Serial.print("Motor 2 generating "); Serial.print(receivedSteps); Serial.println(" microsteps...");
  
  for (long i = 0; i < receivedSteps; i++) {
    digitalWrite(STEP_PIN_2, HIGH);
    delayMicroseconds(800);  // Adjust speed
    digitalWrite(STEP_PIN_2, LOW);
    delayMicroseconds(800);  // Adjust speed
    
    // Progress indicator for large movements
    if (receivedSteps > 1000 && (i + 1) % 50 == 0) {
      Serial.print("Motor 2 progress: "); Serial.print(i + 1); 
      Serial.print("/"); Serial.println(receivedSteps);
    }
  }
  
  Serial.println("Motor 2 movement completed");
}