#include <Arduino.h>
#include <SoftwareSerial.h>

float kp = 1;
#define KPS0001 '1'
#define KDS0001 '2'
#define KIS0001 '3'
#define KPR0001 '4'
#define KDR0001 '5'
#define KIR0001 '6'
#define KPS001 '7'
#define KDS001 '8'
#define KIS001 '9'
#define KPR001 '!'
#define KDR001 '@'
#define KIR001 '#'
#define KPS01 '$'
#define KDS01 '%'
#define KIS01 '^'
#define KPR01 '&'
#define KDR01 '*'
#define KIR01 '('


SoftwareSerial bt(11, 10);

float kp = 0.12;
float kd = 0.15;
float ki = 0.3;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("listo");
  bt.begin(9600);
}

void loop()
{
  if (bt.available())
  {
    Serial.write(bt.read());
    char in = bt.read();
    // in.trim();

    if (in == KPS0001){
      Serial.println("se subio kp");
      kp+=0.001;
    }
    if (in == KDS0001){
      Serial.println("se subio kd");
    kd+=0.001;
    }
    if (in == KIS0001){
      Serial.println("se subio ki");
    ki+=0.001;
    }
    if (in == KPR0001){
      Serial.println("se bajo kp");
    kp-=0.001;
    }
    if (in == KDR0001){
      Serial.println("se bajo kd");
      kd-=0.001;
    }
    if (in == KIR0001){
      Serial.println("se bajo ki");
     ki -= 0.001;
    }
    if (in == KPS001){
      Serial.println("se subio kp 0.01");
      kp+=0.01;
    }
    if (in == KDS001){
      Serial.println("se subio kd 0.01");
    kd+=0.01;
    }
    if (in == KIS001){
      Serial.println("se subio ki 0.01");
     ki+=0.01;
    }
      /////////////////////////////////////////
      if (in == KPR001){
      Serial.println("se bajo kp 0.01");
      kp-=0.01;
      }
    if (in == KDR001){
      Serial.println("se bajo kd 0.01");
      kd-=0.01;
    }
    if (in == KIR001){
      Serial.println("se bajo ki 0.01");
      ki-=0.01;
    }
    if (in == KPS01){
      Serial.println("se subio kp 0.1");
      kp+=0.1;
    }
    if (in == KDS01){
      Serial.println("se subio kd 0.1");
      kd+=0.1;
    }
    if (in == KIS01){
      Serial.println("se subio ki 0.1");
      ki+=0.1;
    }
    if (in == KPR01){
      Serial.println("se bajo kp 0.1");
      kp-=0.1;
    }
    if (in == KDR01){
      Serial.println("se bajo kd 0.1");
      kd-=0.1;
    }
    if (in == KIR01){
      Serial.println("se bajo ki 0.1 ");
      ki-=0.1;
    }
  }

  if (Serial.available())
    bt.write(Serial.read());
  // bt.println(kp);
}
