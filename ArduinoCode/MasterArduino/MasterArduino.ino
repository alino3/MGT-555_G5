#include <Wire.h>

// Motor 1 pins (THIS ARDUINO CONTROLS MOTOR 1)
#define STEP_PIN_1 2
#define DIR_PIN_1 3
#define ENABLE_PIN_1 4

// I2C addresses for other Arduinos
#define ARDUINO_2_ADDRESS 8
#define ARDUINO_3_ADDRESS 9

void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize I2C as master
  Wire.begin();
  
  // Initialize motor pins
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  
  // Enable motor (LOW = enabled)
  digitalWrite(ENABLE_PIN_1, LOW);
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(DIR_PIN_1, LOW);
  
  Serial.println("Arduino 1 Ready - 3 Motor Mode");
  Serial.println("Waiting for commands...");
}

void loop() {
  // Check for serial data: 1 byte command + (3 pulses * 4 bytes) + (3 directions * 1 byte) = 16 bytes
  if (Serial.available() >= 16) {
    byte command = Serial.read();
    
    if (command == 0x01) {
      // Read data for 3 motors: 3 pulses (12 bytes) + 3 directions (3 bytes)
      long steps1 = readLong();
      long steps2 = readLong();
      long steps3 = readLong();
      byte dir1 = Serial.read();
      byte dir2 = Serial.read();
      byte dir3 = Serial.read();
      
      Serial.println("=== RECEIVED 3-MOTOR COMMAND ===");
      Serial.print("Motor 1: "); Serial.print(steps1); Serial.print(" steps, dir: "); Serial.println(dir1);
      Serial.print("Motor 2: "); Serial.print(steps2); Serial.print(" steps, dir: "); Serial.println(dir2);
      Serial.print("Motor 3: "); Serial.print(steps3); Serial.print(" steps, dir: "); Serial.println(dir3);
      
      // Move Motor 1 locally
      if (steps1 != 0) {
        Serial.println("Moving Motor 1...");
        moveMotor(STEP_PIN_1, DIR_PIN_1, steps1, dir1);
      }
      
      // Send to Arduino 2 via I2C (Motor 2)
      if (steps2 != 0) {
        Serial.println("Sending to Arduino 2 (Motor 2) via I2C...");
        sendToSlave(ARDUINO_2_ADDRESS, steps2, dir2);
      }
      
      // Send to Arduino 3 via I2C (Motor 3)
      if (steps3 != 0) {
        Serial.println("Sending to Arduino 3 (Motor 3) via I2C...");
        sendToSlave(ARDUINO_3_ADDRESS, steps3, dir3);
      }
      
      Serial.println("=== ALL COMMANDS SENT ===");
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

void sendToSlave(int address, long steps, byte direction) {
  Wire.beginTransmission(address);
  
  // Send steps (4 bytes)
  Wire.write((byte)((steps >> 24) & 0xFF));
  Wire.write((byte)((steps >> 16) & 0xFF));
  Wire.write((byte)((steps >> 8) & 0xFF));
  Wire.write((byte)(steps & 0xFF));
  
  // Send direction (1 byte)
  Wire.write(direction);
  
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.print("I2C sent to address "); Serial.print(address);
    Serial.print(": "); Serial.print(steps); Serial.print(" steps, dir: "); Serial.println(direction);
  } else {
    Serial.print("I2C ERROR sending to address "); Serial.println(address);
  }
}

void moveMotor(int stepPin, int dirPin, long steps, byte direction) {
  digitalWrite(dirPin, direction);
  delay(10);
  
  Serial.print("Generating "); Serial.print(steps); Serial.println(" steps...");
  
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }
  
  Serial.println("Motor movement completed");
}