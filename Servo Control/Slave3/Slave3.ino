#include <Servo.h>
#include <SPI.h>
#define MISO 12
#define MOSI 11
#define SCK 13
#define SS 10
#define servoPin 4

Servo myservo;
byte spi_receiver=90;
void setup ()
{
  myservo.attach(servoPin);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);  //turn on SPI in slave mode
  SPCR |= _BV(SPIE); // turn on interrupts
  SPI.attachInterrupt;
  Serial.begin(9600);
}
ISR(SPI_STC_vect)
{
   if(SPDR>120)
   spi_receiver=120;
   else if(SPDR<60)
   spi_receiver=60;
   else
   spi_receiver=SPDR;
}

void loop ()
{

int servoPos=120; //60=<servoPos<=120
myservo.write(spi_receiver);
Serial.println(myservo.read());
Serial.println(SPDR);
      if (digitalRead(SS) == HIGH)
    {
        SPDR = 0;
    }


}
