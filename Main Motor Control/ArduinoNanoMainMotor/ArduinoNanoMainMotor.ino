#include <PID_v1.h>
#include <Encoder.h>

#define IN1 8
#define IN2 7
#define PWM 5
#define STBY 4
#define A 2 //Encoder A
#define B 3 //Encoder B
#define AIN1 8
#define AIN2 7

//Declare encoder function
Encoder Enc(A, B);

//Time variables
long previousMillis = 0;
long currentMillis = 0;

//Encoder variables
volatile long currentEncoder;
volatile long previousEncoder = 0;
volatile long oldPosition = 0;
volatile long newPosition;

long positionMain  = -999;
int rotation = 0;
float old_rot_speed = 0;
const int Encoder_1_round = 44; //define number of pulses in one round of encoder


float rot_speed;           //rotating speed in rad/s
const int interval = 5; //choose interval is 1 second (1000 milliseconds)

//Set PID constants
double kp = 15.8526, ki = 1802.8, kd = 0.0096;
double input = 0, output = 0, setpoint = 0;

//Declare PID functions
PID MainPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void PID_setup(void)
{
  //PID initial settings
  MainPID.SetMode(AUTOMATIC);
  MainPID.SetSampleTime(0.005);
  MainPID.SetOutputLimits(0, 255);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(57600); //Set the band rate to your Bluetooth module.
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(A, INPUT_PULLUP);
  pinMode(B,  INPUT_PULLUP);
  Serial.println("Speed Test");
  PID_setup();
}
float read_speed(void)
{
  //read velocity of selected motor
  //return velocity in rad/s

  currentEncoder = Enc.read();
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    rot_speed = (float)((currentEncoder - previousEncoder) * 100 * 60 / (Encoder_1_round));
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
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (direct == -1)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  if (rotation == 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM, 0);
  }
}

void control_PID(float u)
{
  //control the wheel using PID
  int u_sign = sign_of(u); //Get sign of rotating velocity of wheels
  u = abs(u);              //get the absolute value of input speed (because we have the variable u_sign)
  setpoint = u;
  input = read_speed();
  MainPID.Compute();
  //  //Mapping value
  //  float M_in_min = 0;
  //  //M_in_max = 19;
  //  float M_out_min = 0;
  //  float M_out_max = 255;
  //
  //    //Remapping motors' value
  //    u1 = (u1 - M_in_min) * (M_out_max - M_out_min) / (M_in_max - M_in_min) + M_out_min;
  w(output, u_sign);
}
int sign_of(float x)
{
  if (x >= 0)
    return 1;
  else
    return -1;
}

void loop() {
  digitalWrite(STBY, HIGH);
 w(25, 1);
// delay(2000);
 //delay(5000);
//control_PID(9000);
  float new_rot_speed;
  new_rot_speed = read_speed();
  if (new_rot_speed != old_rot_speed) {
    Serial.print("Speed=\t");
    Serial.print(new_rot_speed);
    Serial.print("\t");
    Serial.print(millis());
    Serial.println();
    old_rot_speed = new_rot_speed;
  }
}
