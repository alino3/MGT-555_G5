#include <Wire.h>
#include <Servo.h>

#define STEP_PIN_2 2
#define DIR_PIN_2 3
#define ENABLE_PIN_2 4

// Servo pins
#define SERVO_PHI_PIN 5
#define SERVO_GRIPPER_PIN 6

// Create servo objects
Servo servoPhi;
Servo servoGripper;

// Servo positions
int servoPhi_pos = 90;
int servoGripper_pos = 90;

// Motor data
long receivedSteps = 0;
byte receivedDirection = 0;
bool newData = false;

// Servo data
byte receivedServoPhi = 0xFF;
byte receivedServoGripper = 0xFF;
bool newServoData = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW);
  
  // Attach servos to pins
  servoPhi.attach(SERVO_PHI_PIN);
  servoGripper.attach(SERVO_GRIPPER_PIN);
  
  // Move servos to default position
  servoPhi.write(servoPhi_pos);
  servoGripper.write(servoGripper_pos);
  
  // Initialize I2C as slave with address 9
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
  
  Serial.println("Arduino 3 Ready - Fixed I2C Communication");
  Serial.println("Waiting for commands...");
}

void loop() {
  if (newData) {
    Serial.println("=== EXECUTING STEPPER COMMAND ===");
    Serial.print("Motor Z: "); Serial.print(receivedSteps); 
    Serial.print(" microsteps, dir: "); Serial.println(receivedDirection);
    
    moveMotorZ();
    newData = false;
    
    Serial.println("=== STEPPER COMMAND COMPLETED ===");
  }
  
  if (newServoData) {
    Serial.println("=== EXECUTING SERVO COMMAND ===");
    
    if (receivedServoPhi != 0xFF) {
      Serial.print("Setting Servo Phi to: "); Serial.println(receivedServoPhi);
      setServoPhi(receivedServoPhi);
    }
    
    if (receivedServoGripper != 0xFF) {
      Serial.print("Setting Servo Gripper to: "); Serial.println(receivedServoGripper);
      setServoGripper(receivedServoGripper);
    }
    
    newServoData = false;
    Serial.println("=== SERVO COMMAND COMPLETED ===");
  }
}

void receiveEvent(int bytes) {
  Serial.print("I2C received "); Serial.print(bytes); Serial.println(" bytes");
  
  if (bytes >= 1) {
    byte firstByte = Wire.read();
    
    if (firstByte == 0xBB && bytes >= 6) {
      // Combined command: 0xBB + 4 bytes steps + 1 byte direction + 3 bytes servo data
      processStepperCommand();
      
      // Check if there are more bytes for servos
      if (bytes == 9) {
        processServoCommand();
      }
    }
    else if (firstByte == 0xAA && bytes == 3) {
      // Servo-only command: 0xAA + phi + gripper
      processServoCommand();
    }
    else {
      // Unknown command - read remaining bytes to clear buffer
      while (Wire.available()) {
        Wire.read();
      }
      Serial.println("Unknown command format");
    }
  }
}

void processStepperCommand() {
  // Read steps (4 bytes)
  receivedSteps = 0;
  for (int i = 0; i < 4; i++) {
    if (Wire.available()) {
      receivedSteps = (receivedSteps << 8) | Wire.read();
    }
  }
  
  // Read direction (1 byte)
  if (Wire.available()) {
    receivedDirection = Wire.read();
  }
  
  newData = true;
  Serial.print("Stepper command: "); Serial.print(receivedSteps); 
  Serial.print(" steps, dir: "); Serial.println(receivedDirection);
}

void processServoCommand() {
  // Read servo data (2 bytes after marker)
  if (Wire.available() >= 2) {
    receivedServoPhi = Wire.read();
    receivedServoGripper = Wire.read();
    newServoData = true;
    
    Serial.print("Servo command: φ="); Serial.print(receivedServoPhi);
    Serial.print("°, gripper="); Serial.print(receivedServoGripper); Serial.println("°");
  }
}

void setServoPhi(byte angle) {
  if (angle >= 0 && angle <= 180) {
    servoPhi_pos = angle;
    servoPhi.write(servoPhi_pos);
    Serial.print("Servo φ set to: "); Serial.print(angle); Serial.println("°");
  } else {
    Serial.println("Servo φ angle out of range (0-180)");
  }
}

void setServoGripper(byte angle) {
  if (angle >= 0 && angle <= 180) {
    servoGripper_pos = angle;
    servoGripper.write(servoGripper_pos);
    Serial.print("Servo gripper set to: "); Serial.print(angle); Serial.println("°");
  } else {
    Serial.println("Servo gripper angle out of range (0-180)");
  }
}

void moveMotorZ() {
  digitalWrite(DIR_PIN_2, receivedDirection);
  delay(10);
  
  Serial.print("Motor Z generating "); Serial.print(receivedSteps); Serial.println(" microsteps...");
  
  for (long i = 0; i < receivedSteps; i++) {
    digitalWrite(STEP_PIN_2, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN_2, LOW);
    delayMicroseconds(800);
  }
  
  Serial.println("Motor Z movement completed");
}