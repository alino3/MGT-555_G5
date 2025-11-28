#include <Wire.h>

// Motor 1 pins
#define STEP_PIN_1 2
#define DIR_PIN_1 3
#define ENABLE_PIN_1 4

// I2C addresses
#define ARDUINO_2_ADDRESS 8  // Motor 2
#define ARDUINO_3_ADDRESS 9  // Z-Stepper + Servos

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize motor pins
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW);
  
  Serial.println("=== MASTER ARDUINO SIMPLIFIED ===");
  Serial.println("Ready for commands...");
}

void loop() {
  if (Serial.available() >= 18) {
    byte command = Serial.read();
    
    if (command == 0x01) {
      // Read all data
      long steps1 = readLong();
      long steps2 = readLong();
      long stepsZ = readLong();
      byte dir1 = Serial.read();
      byte dir2 = Serial.read();
      byte dirZ = Serial.read();
      byte servo_phi = Serial.read();
      byte servo_gripper = Serial.read();
      
      Serial.println("\n=== PROCESSING COMMAND ===");
      Serial.print("Z-axis: "); Serial.print(stepsZ); Serial.print(" steps, dir: "); Serial.println(dirZ);
      
      // Move Motor 1 locally
      if (steps1 != 0) {
        moveStepperLocal(steps1, dir1);
      }
      
      // Send to Arduino 2 (Motor 2)
      if (steps2 != 0) {
        sendToArduino2(steps2, dir2);
      }
      
      // Send to Arduino 3 (Z-Stepper + Servos)
      sendToArduino3(stepsZ, dirZ, servo_phi, servo_gripper);
      
      Serial.println("=== COMMAND COMPLETED ===");
    }
  }
}

long readLong() {
  long value = 0;
  for (int i = 0; i < 4; i++) {
    value = (value << 8) | Serial.read();
  }
  return value;
}

void sendToArduino3(long stepsZ, byte dirZ, byte servo_phi, byte servo_gripper) {
  Serial.println("Sending to Arduino 3...");
  
  Wire.beginTransmission(ARDUINO_3_ADDRESS);
  
  // Send stepper data (5 bytes)
  for (int i = 3; i >= 0; i--) {
    byte b = (stepsZ >> (8 * i)) & 0xFF;
    Wire.write(b);
  }
  Wire.write(dirZ);
  
  // Send servo data (3 bytes)
  Wire.write(0xAA);
  Wire.write(servo_phi);
  Wire.write(servo_gripper);
  
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("Command sent to Arduino 3 successfully");
  } else {
    Serial.print("I2C error: ");
    Serial.println(error);
  }
}

void sendToArduino2(long steps, byte direction) {
  Wire.beginTransmission(ARDUINO_2_ADDRESS);
  for (int i = 3; i >= 0; i--) {
    Wire.write((steps >> (8 * i)) & 0xFF);
  }
  Wire.write(direction);
  Wire.endTransmission();
}

void moveStepperLocal(long steps, byte direction) {
  digitalWrite(DIR_PIN_1, direction);
  delay(10);
  
  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN_1, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN_1, LOW);
    delayMicroseconds(800);
  }
}