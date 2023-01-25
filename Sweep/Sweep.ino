#include <Servo.h>
Servo myservo;
void setup() 
{
  myservo.attach(7);
}
void loop() 
{
  myservo.write(30);
  delay(5000);
  myservo.write(120);
  delay(500);
}
