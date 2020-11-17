#include <Encoder.h>

#define IN1 7
#define IN2 8
#define PWM 5
#define EN_A 2
#define EN_B 3

//Declare encoder function
Encoder Enc(EN_A, EN_B);
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); //Set the band rate to your Bluetooth module.
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN_A, INPUT_PULLUP);
  pinMode(EN_B,  INPUT_PULLUP); 
}

void w(int rotation, int direct)
{
    //Control rotation of motor
    //variable "rotation" gets value from 0 (0% power) to 255 (100% power)
    analogWrite(PWM, rotation);
    if (direct == 1)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else if (direct == -1)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
}

void loop() {
  // put your main code here, to run repeatedly:
w(255, 1);

  delay(200);
  w(0, 1);
  
  delay(600);
  w(255, -1);

  delay(200);
  w(0, 1);
   delay(2000);
}
