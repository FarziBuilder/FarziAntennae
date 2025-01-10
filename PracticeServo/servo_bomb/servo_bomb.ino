/*
     Servo Motor Control using the ESP32Servo Library
           Adapted for ESP32
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  // Attach the servo to GPIO pin 9 (change if necessary for your ESP32 board)
  myservo.attach(2, 500, 2500);  // (pin, min pulse width, max pulse width in microseconds)
  myservo.write(96);
}

void loop() {
  // myservo.write(0);  // tell servo to go to a particular angle 
  // delay(1000);
  
  // myservo.write(90);              
  // delay(500); 
  
  // myservo.write(135);              
  // delay(500);
  
  // myservo.write(180);              
  // delay(1500);                     
}
