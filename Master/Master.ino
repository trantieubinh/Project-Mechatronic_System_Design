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

int CB1=A0;
int CB2=A1;
int CB3=A2;
int CB4=A3;
int CB5=A4;
int CB6=A5;
int CB7=A6;
float S1, S2, S3, S4, S5, S6, S7, e;

void setup() {
  Serial.begin(9600);
  pinMode(SS, OUTPUT);
  pinMode(SS1, OUTPUT);
  pinMode(SS2, OUTPUT);
  pinMode(SS3, OUTPUT);
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT);  
  pinMode(A2,INPUT);  
  pinMode(A3,INPUT);  
  pinMode(A4,INPUT);  
  pinMode(A5,INPUT);   
  pinMode(A6,INPUT); 
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
//  sendData(70,2);
//  delay(1000);
//  sendData(40,3);
//  delay(1000);
//  sendData(140,3);
//  delay(1000);
//sendData(90,3);
//sendData(0,2);
  float S1 = analogRead(CB1);
  float S2 = analogRead(CB2);
  float S3 = analogRead(CB3);
  float S4 = analogRead(CB4);
  float S5 = analogRead(CB5);
  float S6 = analogRead(CB6);
  float S7 = analogRead(CB7);// calib sensor
//  S1 = 110 + ((950-140)/(964-150))*(155-150);
//  S2 = 110 + ((950-140)/(950-128))*(145-128);
//  S3 = 110 + ((950-140)/(960-148))*(142-148);
//  S4 = 110 + ((950-140)/(964-147))*(152-147);
//  S5 = 110 + ((950-140)/(954-127))*(141-127);
//  S6 = 110 + ((950-140)/(951-127))*(132-127);
//  S7 = 110 + ((950-140)/(941-147))*(155-147);

      e  = (3*(S7-S1)+2*(S6-S2)+(S5-S3))/(S1+S2+S3+S4+S5+S6+S7); // thuật toán tính e

//      if ((S1+S7) > 2000 )                                        //gặp ngã giao
//          e = 0;                                                  // master quẹo phải trong ... giây
//      else if ((S4+S5+S6+S7) > 4100 && S6 > 900)                  // gặp ngã ba 
//          e = 1;                                                  // master cho đi thẳng ... giây khi gặp ngã ba
//      else if ((S1+S2+S3+S4+S5+S6+S7) > 100  )                    //gặp ngã giao lần 2
//          e = 2;                                                  // master cho đi thẳng ... giây khi gặp ngã giao lần 2
//      else if ((S1+S2+S3+S4) > 4100 && S1 > 900)                  //gặp ngã giao lần 3
//          e = 3;                                                  // master cho xe đi thẳng ... giây
//      else if ((S1+S2+S3+S4+S5+S6+S7) > 100  )                    // gặp giao lộ lần cuối
//          e = 4;                                                  // xe quẹo trái ... giây rồi đi về đích
//    if (S1>100)     Serial.println('A');
//    else     
Serial.println(S1);
delay(1);


}
