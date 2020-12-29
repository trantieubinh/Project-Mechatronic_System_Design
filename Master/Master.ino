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

void setup() {
  Serial.begin(9600);
  pinMode(SS,OUTPUT);
  pinMode(SS1, OUTPUT);
  pinMode(SS2, OUTPUT);
  pinMode(SS3, OUTPUT);
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);
  digitalWrite(SS3, HIGH);
  SPCR |= _BV(MSTR);
  SPI.begin ();
}

void loop() {
  long value=2570;
  int sending=map(value,0,3000,0,255);
digitalWrite(SS2, LOW);
spi_receiver_2 = SPI.transfer (sending); 
digitalWrite(SS2, HIGH);

digitalWrite(SS3, LOW);
   spi_receiver_3 = SPI.transfer (110); 
digitalWrite(SS3, HIGH);
     
  
  /*digitalWrite(ss1, LOW);
    digitalWrite(ss2, LOW);
    spi_receiver_1 = SPI.transfer ('A');
    spi_receiver_2 = SPI.transfer ('A');
    delay(1500);
    spi_receiver_1 = SPI.transfer ('C');
    spi_receiver_2 = SPI.transfer ('C');
    delay(1000);
    spi_receiver_1 = SPI.transfer ('A');
    spi_receiver_2 = SPI.transfer ('A');
    delay(1500);
    spi_receiver_1 = SPI.transfer ('C');
    spi_receiver_2 = SPI.transfer ('C');
    delay(1000);
    digitalWrite(ss1, HIGH);
    digitalWrite(ss2, HIGH);*/
}
