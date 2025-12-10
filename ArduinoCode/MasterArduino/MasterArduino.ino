#include <Wire.h>

// Motor 1 pins (Local)
#define STEP_PIN_1 2
#define DIR_PIN_1 3
#define ENABLE_PIN_1 4

// I2C addresses
#define ARDUINO_3_ADDRESS 9  

void setup() {
  Serial.begin(9600);
  Wire.begin(); 
  Wire.setWireTimeout(3000, true); // FIX: Stop I2C from hanging forever (3ms timeout)

  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW);
  
  // Flash LED to show reset
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); delay(500); digitalWrite(13, LOW);

  Serial.println("=== MASTER ONLINE ===");
}

void loop() {
  if (Serial.available() >= 18) {
    byte header = Serial.peek(); // Peek at the first byte without removing it
    
    // FIX: Buffer Synchronization
    if (header != 0x01) {
      // If the first byte isn't our header, we are out of sync!
      Serial.print("SYNC ERROR: Found "); Serial.println(header, HEX);
      while(Serial.available()) Serial.read(); // Dump the garbage
      Serial.println("Buffer flushed");
      return; 
    }

    // Read the Header
    Serial.read(); 
    
    // Read Data
    long steps1 = readLong();
    long steps2 = readLong(); // Ignore Motor 2
    long stepsZ = readLong();
    byte dir1 = Serial.read();
    byte dir2 = Serial.read(); // Ignore Motor 2
    byte dirZ = Serial.read();
    byte servo_phi = Serial.read();
    byte servo_gripper = Serial.read();
    
    // DEBUG: Tell Python we got the data immediately!
    Serial.println("ACK: Data Received"); 

    // 1. Send to Arduino 3 (I2C)
    sendToArduino3(stepsZ, dirZ, servo_phi, servo_gripper);
    
    // 2. Move Local Motor
    if (steps1 != 0) {
      Serial.print("Moving M1: "); Serial.println(steps1);
      moveStepperLocal(steps1, dir1);
    }
    
    Serial.println("DONE");
  }
}

long readLong() {
  long value = 0;
  for (int i = 0; i < 4; i++) {
    // Wait slightly if serial is slow
    while(!Serial.available()); 
    value = (value << 8) | Serial.read();
  }
  return value;
}

void sendToArduino3(long stepsZ, byte dirZ, byte servo_phi, byte servo_gripper) {
  Wire.beginTransmission(ARDUINO_3_ADDRESS);
  Wire.write(0xBB); // Header
  for (int i = 3; i >= 0; i--) Wire.write((stepsZ >> (8 * i)) & 0xFF);
  Wire.write(dirZ);
  Wire.write(0xAA); // Servo Header
  Wire.write(servo_phi);
  Wire.write(servo_gripper);
  
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("I2C Success");
  } else {
    // If you see this error, check your wires!
    Serial.print("I2C FAILED! Error: "); Serial.println(error);
    // Error 2 = NACK (Slave not found / wrong address)
    // Error 3 = NACK on data
    // Error 5 = Timeout (Wires loose)
  }
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