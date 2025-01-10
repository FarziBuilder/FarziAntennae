#include <Servo.h>

Servo myservo; // create servo object to control a servo

void setup() {
  // Attach the servo to GPIO pin 3 (or another PWM-capable pin)
  myservo.attach(2); // Change pin as per your 
  Serial.begin(9600);
  // myservo.write(90);
  // delay(1000);
  // Serial.println("You have 5 seconds, orient it ");
  // delay(5000);
}

void loop() {
  //Sweep the servo from 0° to 180° in steps of 10°
  // for (float i = 25; i <= 150; i += 0.5) {
  //   myservo.write(i);   // Set servo position
  //   delay(100);        // Wait for the servo to reach the position
  //   Serial.print("i Angle: ");
  //   Serial.println(i);
  // }
  myservo.write(75);
  delay(1000);

}
