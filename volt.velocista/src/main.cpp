#include <Arduino.h>
#include <Motor.h>
#include <QTRSensors.h>

#define DEGUB false

#define BUTTON 4

#define MOTOR_DIR_DER 11
#define MOTOR_PWM_DER 10

#define MOTOR_DIR_IZQ 9
#define MOTOR_PWM_IZQ 6

#define LED_BUILTIN 13

int velocidadGanancia;
int velocidadPerdida;

int velocidadPunta = 200;

int maxSpeed = 230;

int minSpeed = 5;

unsigned long speed;

float proporcional ;
float derivativa ;
float integral ;

int setPoint = 2000;

int setPointMax = 2400;
int setPointMin = 1800;

float kp = 0.01;
float kd = 0;
float ki = 0;

float last = 0;

int arranque = 30;

bool btnPress;

int position;

QTRSensors qtr;

const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];

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
  Serial.begin(9600);
  Calibration();
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
  delay(500);
}

bool inicio = true;

void loop()
{
  btnPress = digitalRead(BUTTON) == 0;
  int position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position

  /*for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);*/
  if (btnPress)
  {
    if (true)
    {
      Serial.println("iniciado");
      if (inicio)
      {
        Serial.println("velocidad de inicio");
        motorDer->GoAvance(30);
        motorIzq->GoAvance(30);
      }

      while (true)
      {
        inicio = false;
        position = qtr.readLineBlack(sensorValues);
        Serial.println("posision : ");
        Serial.println(position);
        proporcional = (position) - (setPoint);
        Serial.println("proporcional : ");
        Serial.println(proporcional);

        derivativa = (proporcional - last);

        // integral = (proporcional + last);

       float  speed = (proporcional * kp) + (derivativa * kd) + (integral * ki);
        Serial.println("speed : ");
        Serial.println(speed);
       int  velocidadGanancia = (velocidadPunta + speed);
       int  velocidadPerdida = (velocidadPunta - speed);
        if (velocidadGanancia > maxSpeed) velocidadGanancia = maxSpeed;
        if (velocidadPerdida < minSpeed) velocidadPerdida = minSpeed;
        bool doblarDer = position > setPointMax;
        bool doblarIzq = position < setPointMin;  
        bool recta = (!doblarDer && !doblarIzq);
        if (doblarDer)
        {
          Serial.println("caso doblar Derecha");
          Serial.println("velocidad motor derecho: ");
          Serial.println(velocidadPerdida);
          Serial.println("velocidad motor izquierdo: ");
          Serial.println(velocidadGanancia);
          // motorDer->GoAvance(velocidadGanancia);
          // motorIzq->GoAvance(velocidadPerdida);
        }
        if (doblarIzq)
        {
          Serial.println("caso doblar izquierda");
          Serial.println("velocidad motor derecho ");
          Serial.println(velocidadPerdida); //cambiar esto yaaaaa los motores
          Serial.println("velocidad motor izquierdo ");
          Serial.println(velocidadGanancia);
          // motorDer->GoAvance(velocidadGanancia);
          // motorIzq->GoAvance(velocidadPerdida);
        }
        if (recta)
        {
          Serial.println(" caso recta");
          Serial.println(velocidadPunta);
          // motorDer->GoAvance(velocidadPunta);
          // motorIzq->GoAvance(velocidadPunta);
        }

        last = proporcional;

        position = qtr.readLineBlack(sensorValues);
        Serial.println("posision : ");
        Serial.println(position);

        delay(1000);
      }
    }
  }
}
