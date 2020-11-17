#include <Encoder.h>

#define IN1 7
#define IN2 8
#define PWM 5
#define EN_A 2
#define EN_B 3

//Declare encoder function
Encoder Enc(EN_A, EN_B);

//Time variables
long previousMillis = 0;
long currentMillis = 0;

//Encoder variables
volatile long currentEncoder;
volatile long previousEncoder = 0;
volatile long oldPosition = 0;
volatile long newPosition;

long positionMain  = -999;
int rotation=0;
float old_rot_speed=0;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); //Set the band rate to your Bluetooth module.
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN_A, INPUT_PULLUP);
  pinMode(EN_B,  INPUT_PULLUP); 
  Serial.println("TwoKnobs Encoder Test:");
}
float read_speed(void)
{
    //read velocity of selected motor
    //return velocity in rad/s
    const int Encoder_1_round = 44; //define number of pulses in one round of encoder
    currentEncoder = Enc.read();
    
    float rot_speed;           //rotating speed in rad/s
    const int interval = 1000; //choose interval is 1 second (1000 milliseconds)
    currentMillis = millis();

    if (currentMillis - previousMillis > interval)
    {
        previousMillis = currentMillis;
        rot_speed = (float)((currentEncoder - previousEncoder) * 2 * PI / Encoder_1_round);
        previousEncoder = currentEncoder;
        return rot_speed;
    }
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
//  w(255, 1);
//  delay(200);
//  w(0, 1);
//  delay(600);
//  w(255, -1);
//  delay(200);
//  w(0, 1);
//   delay(2000);
//long newMain;
//  newMain = Enc.read();
//  if (newMain != positionMain) {
//    Serial.print("Main = ");
//    Serial.print(newMain);
//   Serial.println();
//    positionMain = newMain;
//  }
//  // if a character is sent from the serial monitor,
//  // reset both back to zero.
//  if (Serial.available()) {
//    Serial.read();
//    Serial.println("Reset both knobs to zero");
//    Enc.write(0);
//  }
w(rotation,1);
float new_rot_speed;
new_rot_speed=read_speed();
if (new_rot_speed != old_rot_speed) {
Serial.print("Speed = ");
Serial.print(new_rot_speed);
Serial.println();
old_rot_speed = new_rot_speed;
}

if (Serial.available())
{
  Serial.read();
  Serial.println("Increase ");
  rotation+=10;
  if (rotation>255) 
{
  rotation = 255;
}
Serial.println(rotation);
}


if (Serial.available()==2)
  rotation=0;


}
