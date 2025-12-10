#include <Servo.h>

// --- PIN DEFINITIONS ---
// Motor 1 (Base) - New Pins
#define STEP_PIN_1 8
#define DIR_PIN_1  9

// Motor 2 (Shoulder) - New Pins
#define STEP_PIN_2 10
#define DIR_PIN_2  11

// Motor 3 (Z-Axis) - From Slave 3
#define STEP_PIN_3 2
#define DIR_PIN_3  3

// Shared Enable Pin (From Slave 3)
#define ENABLE_PIN 4

// Servos - From Slave 3
#define SERVO_PHI_PIN 5
#define SERVO_GRIPPER_PIN 6

// --- OBJECTS & VARIABLES ---
Servo servoPhi;
Servo servoGripper;

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Stepper Pins
  pinMode(STEP_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT); pinMode(DIR_PIN_3, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Enable Drivers (LOW = Enabled)
  digitalWrite(ENABLE_PIN, LOW);

  // 2. Initialize Servos
  servoPhi.attach(SERVO_PHI_PIN);
  servoGripper.attach(SERVO_GRIPPER_PIN);
  
  // Set default positions
  servoPhi.write(90);
  servoGripper.write(90);

  Serial.println("=== SINGLE NANO CONTROLLER READY ===");
}

void loop() {
  // We expect exactly 18 bytes from Python
  // Format: [0x01] [S1-4B] [S2-4B] [S3-4B] [D1] [D2] [D3] [SV1] [SV2]
  if (Serial.available() >= 18) {
    
    // Check Header
    if (Serial.read() != 0x01) {
      // If sync lost, clear buffer
      while(Serial.available()) Serial.read();
      return;
    }

    // --- READ DATA ---
    long steps1 = readLong();
    long steps2 = readLong();
    long steps3 = readLong();
    
    byte dir1 = Serial.read();
    byte dir2 = Serial.read();
    byte dir3 = Serial.read();
    
    byte valServoPhi = Serial.read();
    byte valServoGripper = Serial.read();

    Serial.println("Command Received");

    // --- 1. MOVE SERVOS ---
    servoPhi.write(valServoPhi);
    servoGripper.write(valServoGripper);
    delay(100); // Give servos a moment to start moving

    // --- 2. MOVE STEPPERS (Simultaneously) ---
    moveSimultaneous(steps1, dir1, steps2, dir2, steps3, dir3);

    // --- 3. REPORT COMPLETION ---
    Serial.println("Movement Done");
  }
}

// Helper to read 4 bytes as a long integer
long readLong() {
  long value = 0;
  for (int i = 0; i < 4; i++) {
    while (!Serial.available()); // Wait for byte
    value = (value << 8) | Serial.read();
  }
  return value;
}

// Linear Interpolation Algorithm for smooth simultaneous movement
void moveSimultaneous(long s1, byte d1, long s2, byte d2, long s3, byte d3) {
  
  // Set Directions
  digitalWrite(DIR_PIN_1, d1);
  digitalWrite(DIR_PIN_2, d2);
  digitalWrite(DIR_PIN_3, d3);

  // Determine the "Master" motor (longest distance)
  long maxSteps = s1;
  if (s2 > maxSteps) maxSteps = s2;
  if (s3 > maxSteps) maxSteps = s3;

  // If no movement needed, exit
  if (maxSteps == 0) return;

  Serial.print("Moving steps: "); Serial.println(maxSteps);

  // Counters for Bresenham-like algorithm
  long count1 = 0;
  long count2 = 0;
  long count3 = 0;

  // Loop through the longest distance
  // We pulse the motors proportionally so they start and end together
  for (long i = 0; i < maxSteps; i++) {
    
    // Check Motor 1
    count1 += s1;
    if (count1 >= maxSteps) {
      digitalWrite(STEP_PIN_1, HIGH);
      count1 -= maxSteps;
    }
    
    // Check Motor 2
    count2 += s2;
    if (count2 >= maxSteps) {
      digitalWrite(STEP_PIN_2, HIGH);
      count2 -= maxSteps;
    }
    
    // Check Motor 3
    count3 += s3;
    if (count3 >= maxSteps) {
      digitalWrite(STEP_PIN_3, HIGH);
      count3 -= maxSteps;
    }

    // Pulse Width (Wait)
    delayMicroseconds(10); 
    
    // Pull all STEP pins LOW
    digitalWrite(STEP_PIN_1, LOW);
    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_3, LOW);

    // Speed Control (Delay between steps)
    // Adjust this value (800) to change speed. Lower = Faster.
    delayMicroseconds(600); 
  }
}