// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.write(20);
  delay(5000);
  Serial.begin(9600);
  Serial.println('Beginning...');
} 
 
 
void loop() 
{
  // Find low
  // This is the number that causes the motor to start spinning
  // This is usually around 38
  for (pos = 30; pos < 50; pos += 1) {
    Serial.println(pos);
    myservo.write(pos);
    delay(1000);
  }
  
  // Find high
  // This is the number that causes the motor's speed to be indifferentiable from its speed at 300
  // This is usually around 171
  for (pos = 160; pos < 180; pos += 1) {
    Serial.println(pos);
    myservo.write(pos);
    delay(1000);
    Serial.print("--- "); Serial.println(300);
    myservo.write(300);
    delay(1000);
  }
} 
