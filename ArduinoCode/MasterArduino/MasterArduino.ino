#include <Wire.h>

// Motor 1 pins (THIS ARDUINO CONTROLS MOTOR 1)
#define STEP_PIN_1 2    // → STEP on driver (PUL+)
#define DIR_PIN_1 3     // → DIR on driver (DIR+)
#define ENABLE_PIN_1 4  // → ENABLE on driver

// I2C addresses for other Arduinos
#define ARDUINO_2_ADDRESS 8

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
  
  Serial.println("Arduino 1 Ready - I2C Master");
  Serial.println("Send: steps1,dir1,steps2,dir2");
  Serial.println("Example: 200,1,300,0");
}

void loop() {
  // Check for serial data in format: "200,1,300,0,150,1"
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Parse comma-separated values: steps1,dir1,steps2,dir2,steps3,dir3
    long steps[3];
    byte dirs[3];
    int index = 0;
    int start = 0;
    
    for (int i = 0; i <= input.length(); i++) {
      if (i == input.length() || input.charAt(i) == ',') {
        if (index % 2 == 0) {
          // Even index: steps value
          steps[index/2] = input.substring(start, i).toInt();
        } else {
          // Odd index: direction value
          dirs[index/2] = input.substring(start, i).toInt();
        }
        start = i + 1;
        index++;
      }
    }
    
    if (index == 4) {
      Serial.println("=== RECEIVED COMMAND ===");
      Serial.print("Motor 1: "); Serial.print(steps[0]); Serial.print(" steps, dir: "); Serial.println(dirs[0]);
      Serial.print("Motor 2: "); Serial.print(steps[1]); Serial.print(" steps, dir: "); Serial.println(dirs[1]);
      
      // Move Motor 1 locally
      if (steps[0] > 0) {
        Serial.println("Moving Motor 1...");
        moveMotor(STEP_PIN_1, DIR_PIN_1, steps[0], dirs[0]);
      }
      
      // Send to Arduino 2 via I2C
      if (steps[1] > 0) {
        Serial.println("Sending to Arduino 2 via I2C...");
        sendToSlave(ARDUINO_2_ADDRESS, steps[1], dirs[1]);
      }
      
      Serial.println("=== ALL COMMANDS SENT ===");
    } else {
      Serial.println("ERROR: Invalid format! Use: steps1,dir1,steps2,dir2,steps3,dir3");
    }
  }
}

void sendToSlave(int address, long steps, byte direction) {
  Wire.beginTransmission(address);
  
  // Send steps (4 bytes as long)
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
  // Set direction
  digitalWrite(dirPin, direction);
  delay(10);
  
  // Generate steps (MICROSTEPS - each pulse is one microstep)
  Serial.print("Generating "); Serial.print(steps); Serial.println(" microsteps...");
  
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2500);  // Adjust speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2500);  // Adjust speed
  }
  
  Serial.println("Motor movement completed");
}