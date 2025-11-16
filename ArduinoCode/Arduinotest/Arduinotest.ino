void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Wait for serial port to connect (needed for native USB)
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  Serial.println("Arduino is ready!");
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming string until newline
    String receivedMessage = Serial.readStringUntil('\n');
    
    // Remove any carriage return characters
    receivedMessage.trim();
    
    // Print what was received
    Serial.print("Received: ");
    Serial.println(receivedMessage);
    
    // Check if the message is "hello arduino"
    if (receivedMessage.equals("hello arduino")) {
      // Send response back to PC
      Serial.println("hello PC");
    } else {
      // Echo back any other message with a response
      Serial.println("Message received: " + receivedMessage);
    }
  }
  
  // Small delay to stabilize
  delay(100);
}