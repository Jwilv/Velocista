#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Motor.h>
#include <QTRSensors.h>

#define DEGUB 0

#define BTON 0

#define DEBUG_CALCULOS 0

#define BUTTON 4

#define MOTOR_DIR_DER 11
#define MOTOR_PWM_DER 10

#define MOTOR_DIR_IZQ 9
#define MOTOR_PWM_IZQ 6

#define LED_BUILTIN 13

int velocidadGanancia;
int velocidadPerdida;

int velocidadPunta = 70;

int maxSpeed = 150;

int minSpeed = 20;

float speed;

float proporcional;
float derivativa;
float integral;

//se cree que 33 para una velocidad de 80 
int equiparamiento = 20;



int setPoint = 2000;

int setPointMax = 2200;
int setPointMin = 1800;

float kp = 0.017;
float kd = 0.15;
float ki = 0.001;

float last = 0;

int arranque = 30;

bool btnPress;

int position;

QTRSensors qtr;

const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];

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


SoftwareSerial bt(3, 2);

  void Comunication(){
    if (bt.available())
  {
    char in = bt.read();
    if (in == KPS0001) kp+=0.001;
    if (in == KDS0001) kd+=0.001;
    if (in == KIS0001) ki+=0.001;
    if (in == KPR0001) kp-=0.001;
    if (in == KDR0001) kd-=0.001;
    if (in == KIR0001) ki -= 0.001;
    if (in == KPS001) kp+=0.01;
    if (in == KDS001) kd+=0.01;
    if (in == KIS001) ki+=0.01;
    /////////////////////////////////////////
    if (in == KPR001) kp-=0.01;
    if (in == KDR001) kd-=0.01;
    if (in == KIR001) ki-=0.01;
    if (in == KPS01) kp+=0.1;
    if (in == KDS01) kd+=0.1;
    if (in == KIS01) ki+=0.1;
    if (in == KPR01) kp-=0.1;
    if (in == KDR01) kd-=0.1;
    if (in == KIR01) ki-=0.1;
  }
  }

/////////////////////////////

// calibration

void Calibration()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A6, A5, A4, A3, A2, A1, A0}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //

  // print the calibration minimum values measured when emitters were on
}

// constructors
MOTOR *motorDer = new MOTOR(MOTOR_DIR_DER, MOTOR_PWM_DER);
MOTOR *motorIzq = new MOTOR(MOTOR_DIR_IZQ, MOTOR_PWM_IZQ);

void setup()
{ // configure the sensors


  if (DEBUG_CALCULOS) Serial.begin(9600);
  Calibration();
 // Serial.begin(9600);

  if(BTON)bt.begin(9600);
  if (DEGUB)
  {
    Serial.begin(9600);
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();
  }

  // print the calibration maximum values measured when emitters were on
  if (DEGUB)
  {
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }

  pinMode(BUTTON, INPUT);


 // delay(500);
}

bool inicio = true;

void loop()
{ 
  //Serial.println(digitalRead(BUTTON));
  btnPress = digitalRead(BUTTON) == 0;
  int position = qtr.readLineWhite(sensorValues);
  if(BTON)Comunication();
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
/*
for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

*/
  

  //delay(250);
  if (btnPress)
  {
    if (true)
    {
      //Serial.println("iniciado");
      if (inicio)
      {
        //Serial.println("velocidad de inicio");
        motorDer->GoAvance(30 + equiparamiento);
        motorIzq->GoAvance(30);
      }

      while (true)
      {
        if(BTON)Comunication();
        inicio = false;
        position = qtr.readLineWhite(sensorValues);
        if(DEBUG_CALCULOS){
          Serial.println("posision : ");
        Serial.println(position);
        }
        proporcional = (position) - (setPoint);
        if(DEBUG_CALCULOS){
        Serial.println("proporcional : ");
        Serial.println(proporcional);
        }

        derivativa = (proporcional - last);

        integral = (integral + proporcional);

        float speed = (proporcional * kp) + (derivativa * kd) + (integral * ki);
        if(DEBUG_CALCULOS){
        Serial.println("speed : ");
        Serial.println(speed);
        }
        int velocidadGanancia = (velocidadPunta + speed);
        int velocidadPerdida = (velocidadPunta - speed);
        if (velocidadGanancia > maxSpeed) velocidadGanancia = maxSpeed;
        if (velocidadPerdida < minSpeed) velocidadPerdida = minSpeed;
        bool doblarDer = position > setPointMax;
        bool doblarIzq = position < setPointMin;
        bool recta = (!doblarDer && !doblarIzq);
        if (doblarDer)
        {
          if (DEBUG_CALCULOS)
          {
            bt.println("caso doblar Derecha");
            bt.println("velocidad motor derecho: ");
            bt.println(velocidadPerdida + equiparamiento);
            bt.println("velocidad motor izquierdo: ");
            bt.println(velocidadGanancia);
          }
          motorDer->GoAvance(velocidadPerdida + equiparamiento);
          motorIzq->GoAvance(velocidadGanancia);
        }
        if (doblarIzq)
        { if(DEBUG_CALCULOS){
          bt.println("caso doblar izquierda");
          bt.println("velocidad motor derecho ");
          bt.println(velocidadPerdida + equiparamiento); 
          bt.println("velocidad motor izquierdo ");
          bt.println(velocidadGanancia);
        }
          
          motorDer->GoAvance(velocidadPerdida + equiparamiento);
          motorIzq->GoAvance(velocidadGanancia );
        }
        if (recta){
          if(DEBUG_CALCULOS){
          Serial.println(" caso recta");
          Serial.println(velocidadPunta);
        }
          
          motorDer->GoAvance(velocidadPunta + equiparamiento );
          motorIzq->GoAvance(velocidadPunta);
        }

        last = proporcional;

        position = qtr.readLineWhite(sensorValues);
        if(DEBUG_CALCULOS){
        Serial.println("posision : ");
        Serial.println(position);
        }
        
        if(BTON){
          bt.println("valor kp");
          bt.println(kp);
          bt.println("valor kd");
          bt.println(kd);
          bt.println("valor ki");
          bt.println(ki);
        }
        if(DEBUG_CALCULOS)delay(1000);
      }
    }
  }
  
}


