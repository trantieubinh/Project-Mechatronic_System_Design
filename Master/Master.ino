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
long speedcar;
byte byteSend = 120;
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
//  SPCR |= _BV(MSTR);
  SPI.begin ();
}
int mapping(long speedcar)
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
  sendData(70,2);
  delay(1000);
  sendData(40,3);
  delay(1000);
  sendData(140,3);
  delay(1000);

}
