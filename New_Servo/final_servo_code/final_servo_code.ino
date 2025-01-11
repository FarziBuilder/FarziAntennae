#include <Servo.h>

// -----------------------------
// 1. Servo Objects
// -----------------------------
Servo azServo;  // Azimuth Servo
Servo elServo;  // Elevation Servo

float currentAz = 0.0;  // Current Azimuth angle (degrees)
float currentEl = 0.0;  // Current Elevation angle (degrees)

float midValue = 90.0;

// -----------------------------
// 2. Variables for Parsing
// -----------------------------
String incomingData = "";
bool isReading = false;

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  
  // Attach servos to their respective pins
  azServo.attach(3);   // Azimuth servo on pin 3
  elServo.attach(2);   // Elevation servo on pin 2
  
  // Initialize servo positions
  azServo.write(midValue);
  delay(1000);
  elServo.write(75);
  delay(1000);
  
  Serial.println("You have 5 seconds to orient it.");
  delay(5000);
  
  Serial.println("Ready to receive data.");
}

void loop() {
  // Check if data is available on the serial port
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Read incoming byte
    
    // Check for start marker
    if (receivedChar == '<') {
      incomingData = "";   // Reset the incoming data buffer
      isReading = true;    // Start reading data
      continue;            // Move to the next character
    }
    
    // Check for end marker
    if (receivedChar == '>' && isReading) {
      isReading = false;   // Stop reading data
      processIncomingData(incomingData);  // Process the complete message
      incomingData = "";   // Reset the buffer for the next message
      continue;            // Move to the next character
    }
    
    // If currently reading, append the character to the buffer
    if (isReading) {
      incomingData += receivedChar;
    }
  }
  
  // Add a small delay to stabilize the loop
  delay(10);
}

// -----------------------------
// 3. Function to Process Data
// -----------------------------
void processIncomingData(String data) {
  data.trim();  // Remove any leading/trailing whitespace
  
  // Check if data contains a comma separator
  int commaIndex = data.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("Invalid Input. Please use format <azimuth,elevation>.");
    return;
  }
  
  // Extract azimuth and elevation substrings
  String azStr = data.substring(0, commaIndex);
  String elStr = data.substring(commaIndex + 1);
  
  // Convert substrings to float
  float targetAz = azStr.toFloat();
  float targetEl = elStr.toFloat();
  
  // Validate conversion
  if (azStr.length() == 0 || elStr.length() == 0) {
    Serial.println("Invalid Input. Azimuth or Elevation value missing.");
    return;
  }
  
  // Print received target values
  Serial.print("targetAz: ");
  Serial.println(targetAz, 2);
  Serial.print("targetEl: ");
  Serial.println(targetEl, 2);
  
  // Apply constraints to target angles
  targetAz = (targetAz > 150.0) ? 150.0 : targetAz;
  targetAz = (targetAz < 30.0) ? 30.0 : targetAz;
  targetEl = (targetEl > 150.0) ? 150.0 : targetEl;
  targetEl = (targetEl < 30.0) ? 30.0 : targetEl;
  
  
  // Move servos to target positions
  azServo.write(targetAz);
  delay(1500);  // Wait for servo to reach position
  
  elServo.write(targetEl);
  delay(1500);  // Wait for servo to reach position
  
  Serial.println("Servos reoriented to target angles.");
}
