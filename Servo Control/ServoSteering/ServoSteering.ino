#include <Servo.h>

Servo myservo;
int servoPin = 3;       // Khai báo chân điều khiển servo

void setup ()
{
  myservo.attach(servoPin);
  Serial.begin(9600);
}

void loop ()
{
  int servoPos = 0;
  myservo.write(servoPos);
  Serial.println(servoPos);
}
