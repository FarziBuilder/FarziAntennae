/*
     Dual Continuous Servo Azimuth and Elevation Control
*/

#include <Servo.h>

// -----------------------------
// 1. Servo Objects
// -----------------------------
Servo azServo;  // Azimuth Servo
Servo elServo;  // Elevation Servo

// -----------------------------
// 2. Servo Specifications
// -----------------------------

// Azimuth Servo Specifications
const float AZ_DEG_PER_SEC = 23.0;  // Angular speed for Azimuth (degrees per second)
const int AZ_cwCommand   = 84;       // CW command for Azimuth servo
const int AZ_ccwCommand  = 96;       // CCW command for Azimuth servo

// Elevation Servo Specifications
const float EL_DEG_PER_SEC = 23.0;  // Angular speed for Elevation (degrees per second)
const int EL_cwCommand   = 84;       // CW command for Elevation servo -> will give +ve elevation
const int EL_ccwCommand  = 96;       // CCW command for Elevation servo -> -ve

// Common Command for Stopping Servos
const int stopCommand = 90;

// -----------------------------
// 3. Current Angles
// -----------------------------
float currentAz = 0.0;  // Current Azimuth angle (degrees)
float currentEl = 0.0;  // Current Elevation angle (degrees)

// -----------------------------
// 4. Setup Function
// -----------------------------
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  Serial.println("Dual Continuous Servo Azimuth and Elevation Control");

  // Attach Servos to Pins
  azServo.attach(3);   // Azimuth servo on pin 9
  elServo.attach(2);   // Elevation servo on pin 2

  // Initialize Servos to Stop
  azServo.write(stopCommand);
  elServo.write(stopCommand);
}

// -----------------------------
// 5. Main Loop
// -----------------------------
void loop() {
  // Check if data is available on Serial
  if (Serial.available() > 0) {
    // Read incoming data as a string until newline
    String inputData = Serial.readStringUntil('\n');

    // Trim any leading/trailing whitespace (including \r)
    inputData.trim();

    // Debug: Print the received input
    // Serial.print("Received Raw Input: '");
    // Serial.print(inputData);
    // Serial.println("'");

    // Split the inputData into azimuth and elevation
    int spaceIndex = inputData.indexOf(' ');
    if (spaceIndex == -1) {
      // No space found, invalid input
      Serial.println("Invalid Input. Please send two float values separated by a space (e.g., 45.0 30.0)");
      return;
    }

    // Extract substrings for azimuth and elevation
    String azStr = inputData.substring(0, spaceIndex);
    String elStr = inputData.substring(spaceIndex + 1);

    // Convert substrings to float
    float targetAz = azStr.toFloat();
    float targetEl = elStr.toFloat();

    // Check if both conversions were successful
    if (azStr.length() == 0 || elStr.length() == 0) {
      Serial.println("Invalid Input. Please ensure both azimuth and elevation values are provided.");
      return;
    }

    // Serial.print("Received Targets - Azimuth: ");
    // Serial.print(targetAz);
    Serial.print("°, Elevation: ");
    Serial.print(targetEl);
    Serial.println("°");

    // Rotate Azimuth Servo
    rotateServo(
      azServo,
      currentAz,
      targetAz,
      AZ_DEG_PER_SEC,
      AZ_cwCommand,
      AZ_ccwCommand,
      "Azimuth"
    );

    //Rotate Elevation Servo
    rotateServo(
      elServo,
      currentEl,
      targetEl,
      EL_DEG_PER_SEC,
      EL_cwCommand,
      EL_ccwCommand,
      "Elevation"
    );

    // Confirm Completion
    // Serial.print("Final Angles - Azimuth: ");
    // Serial.print(currentAz);
    // Serial.print("°, Elevation: ");
    // Serial.print(currentEl);
    // Serial.println("°");
  }
}

// -----------------------------
// 6. rotateServo Function
// -----------------------------
void rotateServo(Servo &servo, float &currentAngle, float targetAngle,
                float degPerSec, int cwCmd, int ccwCmd, const char *label) 
{
  // 1. Calculate Angle Difference
  float diff = targetAngle - currentAngle;
  // Serial.println("targetANgle and currentAngle are " + String(targetAngle) + String(currentAngle));
  // 2. Normalize Difference to [-180, 180]
  while (diff > 180.0)  diff -= 360.0;
  while (diff < -180.0) diff += 360.0;

  // 3. Determine Rotation Direction and Angle
  float rotateAngle = fabs(diff);
  float rotateTime  = rotateAngle / degPerSec; // Time in seconds

  // 4. Decide Rotation Direction
  if (diff > 0) {
    // Rotate Clockwise
    servo.write(cwCmd);
    // Serial.print(label);
    // Serial.println(" Servo: Rotating Clockwise");
  } else if (diff < 0) {
    // Rotate Counter-Clockwise
    servo.write(ccwCmd);
    // Serial.print(label);
    // Serial.println(" Servo: Rotating Counter-Clockwise");
  } else {
    // No Rotation Needed
    servo.write(stopCommand);
    // Serial.print(label);
    // Serial.println(" Servo: Already at Target");
    return;
  }

  // 5. Wait for Rotation to Complete
  unsigned long startTime = millis();
  unsigned long waitTime   = (unsigned long)(rotateTime * 1000);
  // Serial.println(waitTime);

  while ((millis() - startTime) < waitTime) {
    // Optionally, add non-blocking code here
  }

  // 6. Stop the Servo
  servo.write(stopCommand);
  // Serial.print(label);
  // Serial.println(" Servo: Stopped");

  // 7. Update Current Angle
  currentAngle = targetAngle;
}
