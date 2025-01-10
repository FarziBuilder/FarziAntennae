#include <Servo.h>

// -----------------------------
// 1. Servo Objects
// -----------------------------
Servo azServo;  // Azimuth Servo
Servo elServo;  // Elevation Servo

float currentAz = 0.0;  // Current Azimuth angle (degrees)
float currentEl = 0.0;  // Current Elevation angle (degrees)

float midValue = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  azServo.attach(3);   // Azimuth servo on pin 3s
  elServo.attach(2);   // Elevation servo on pin 2

  
  azServo.write(midValue);
  delay(1000);
  elServo.write(75);
  delay(1000);
  Serial.println("You have 5 seconds, orient it ");
  delay(5000);
}

void loop() {
   if (Serial.available() > 0){
    String inputData = Serial.readStringUntil('\n');
    Serial.println("This data came in " + inputData);
    inputData.trim();
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

    Serial.print("targetAz: ");
    Serial.println(targetAz);
    Serial.print("targetEl: ");
    Serial.println(targetEl);

    if (azStr.length() == 0 || elStr.length() == 0) {
      Serial.println("Invalid Input. Please ensure both azimuth and elevation values are provided.");
      return;
    }

    if(targetAz > 150){
      Serial.println("Keeping azi locked at 150 degrees. Reorient it");
      targetAz = 150;
    }
    if(targetEl > 150){
      Serial.println("Keeping ele locked at 150 degrees. Reorient it");
      targetAz = 150;
    }

    azServo.write(targetAz);
    delay(1500);
    elServo.write(targetEl);
    delay(1500);
   }  

}
