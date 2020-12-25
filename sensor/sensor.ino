#include <SPI.h> //thư viện SPI.h này là của STM32

#define SCK  PB3
#define MISO PB4
#define MOSI PB5
#define SS   PA15

int CB1 = PA0;
int CB2 = PA1;
int CB3 = PA2;
int CB4 = PA3;
int CB5 = PA4;
int CB6 = PA5;
int CB7 = PA6;
float S1, S2, S3, S4, S5, S6, S7, e;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(CB1, INPUT);
  pinMode(CB2, INPUT);
  pinMode(CB3, INPUT);
  pinMode(CB4, INPUT);
  pinMode(CB5, INPUT);
  pinMode(CB6, INPUT);
  pinMode(CB7, INPUT);

  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  received = false;
  SPI.attachInterrupt();
}

ISR (SPI_STC_vect)                        // Inerrrput routine function
{
  // Value received from master STM32F103C8 is stored in variable slavereceived
  received = true;                        // Sets received as True
}

void loop() {
  // put your main code here, to run repeatedly:
  float S1 = analogRead(CB1);
  float S2 = analogRead(CB2);
  float S3 = analogRead(CB3);
  float S4 = analogRead(CB4);
  float S5 = analogRead(CB5);
  float S6 = analogRead(CB6);
  float S7 = analogRead(CB7);

  // calib sensor
  S1 = 110 + ((950 - 140) / (964 - 150)) * (155 - 150)
       S2 = 110 + ((950 - 140) / (950 - 128)) * (145 - 128)
            S3 = 110 + ((950 - 140) / (960 - 148)) * (142 - 148)
                 S4 = 110 + ((950 - 140) / (964 - 147)) * (152 - 147)
                      S5 = 110 + ((950 - 140) / (954 - 127)) * (141 - 127)
                           S6 = 110 + ((950 - 140) / (951 - 127)) * (132 - 127)
                                S7 = 110 + ((950 - 140) / (941 - 147)) * (155 - 147)

                                     e  = (3(S7 - S1) + 2(S6 - S2) + (S5 - S3)) / (S1 + S2 + S3 + S4 + S5 + S6 + S7); // thuật toán tính e

  if (S1 + S2 + S3 + S4 + S5 + S6 + S7) > //gặp ngã giao
    e = 0;           // master quẹo phải
  else if ((S1 + S2 + S3 + S4 + S5 + S6 + S7) > 4130 && S6 > 900) // gặp ngã ba
    e = 1;            // master cho đi thẳng khi gặp ngã ba
  else if (S1 + S2 + S3 + S4 + S5 + S6 + S7) = //gặp ngã giao lần 2
      e = 1;            // master cho đi thẳng khi gặp ngã giao lần 2
  else if (S1 + S2 + S3 + S4 + S5 + S6 + S7) = //gặp ngã giao lần 3
      e = 2;            // master cho xe quẹo trái về đích

  SPDR = e
}
