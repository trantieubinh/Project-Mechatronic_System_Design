#include <Servo.h>

Servo myservo;
int servoPin = 4;       // Khai báo chân điều khiển servo

void setup ()
{
  myservo.attach(servoPin);
  Serial.begin(9600);
}

void loop ()
{
  int servoPos = 0;
  for (servoPos = 0; servoPos <= 180; servoPos += 5)
  {
    myservo.write(servoPos);
    Serial.println(myservo.read());
    //delay(50);
  }

  for (servoPos = 180; servoPos >= 0; servoPos -= 5)
  {
    myservo.write(servoPos);
    Serial.println(myservo.read());
    //delay(50);
  }
}
