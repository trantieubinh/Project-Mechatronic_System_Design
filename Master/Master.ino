#include <SPI.h>
#define SS3 4
#define SS2 3
#define SS1 2
#define MISO 12
#define MOSI 11
#define SCK 13
byte spi_receiver_1;
byte spi_receiver_2;
byte spi_receiver_3;
long speedcar = 2000;
int byteSend;
char command;
void setup() {
  Serial.begin(9600);
  pinMode(SS, OUTPUT);
  pinMode(SS1, OUTPUT);
  pinMode(SS2, OUTPUT);
  pinMode(SS3, OUTPUT);
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);
  digitalWrite(SS3, HIGH);
  SPCR |= _BV(MSTR);
  SPI.begin ();
}
int sending(long speedcar)
{
  return map(speedcar, 0, 4000, 0, 255);
}
void sendData(int sending, int slave)
{
  switch (slave)
  {
    case 1:
      { digitalWrite(SS1, LOW);
        spi_receiver_1 = SPI.transfer(sending);
        digitalWrite(SS1, HIGH);
        break;
      }
    case 2:
      { digitalWrite(SS2, LOW);
        spi_receiver_2 = SPI.transfer(sending);
        digitalWrite(SS2, HIGH);
        break;

      }
    case 3:
      { digitalWrite(SS3, LOW);
        spi_receiver_2 = SPI.transfer(sending);
        digitalWrite(SS3, HIGH);
        break;

      }
  }
}


void loop() {
  if (Serial.available() > 0)
  {
    sendData(90, 3);
    command = Serial.read();
    switch (command)
    {
      case '0': speedcar = 1000; break;
      case '1': speedcar = 1500; break;
      case '2': speedcar = 2000; break;
      case '3': speedcar = 2500; break;
      case '4': speedcar = 3000; break;
      case '5': speedcar = 3100; break;
      case '6': speedcar = 3200; break;
      case '7': speedcar = 3500; break;
      case '8': speedcar = 3700; break;
      case '9': speedcar = 3900; break;
      case 'q': speedcar = 4000; break;
        byteSend = sending(speedcar);
    }
    switch (command)
    {
      case 'F':
        sendData(byteSend, 2);
        sendData(90, 3);
        break;
      case 'R'://right
        sendData(120, 3);
        break;
      case 'L'://left
        sendData(60, 3);
        break;
      case 'I'://go right
        sendData(byteSend, 2);
        sendData(120, 3);
        break;
      case 'G': //go left
        sendData(byteSend, 2);
        sendData(60, 3);
        break;
      case 'S'://stop
        sendData(0, 2);
        sendData(90, 3);
        break;
    }

  }

}
