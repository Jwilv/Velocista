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

int topSpeed = 70;

float speed;

int proporcional = 0.1;
int derivativa = 0;
int integral = 0;

int setPoint = 3500;

int correcionRueda = 0;

float maxSpeed = 200;

float minSpeed = 15;


int kp = 0;
int kd = 0;
int ki = 0;

int last = 0;

bool btnPress;

int position;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// calibration

void Calibration()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++)
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
  Serial.println("esperando boton");
  if (btnPress)
  {
    Serial.println("iniciado");
    if (inicio)
    {
      Serial.println("velocidad de inicio");
      //motorDer->GoAvance(30);
      //motorIzq->GoAvance(30);
    }

    while (true)
    {
      inicio = false;
      position = qtr.readLineWhite(sensorValues);
      Serial.println("posision : ");
      Serial.println(position);
      proporcional = (position) - (setPoint);
      //Serial.println("proporcional : ");
      //Serial.println(proporcional);

      derivativa = (proporcional - last);
      //Serial.println("derrivativa : ");
      //Serial.println(derivativa);
      integral = (proporcional + last);
      //Serial.println("integral : ");
      //Serial.println(integral);
      speed = (proporcional * kp) + (derivativa * kd) + (integral * ki);
      Serial.println("speed : ");
      Serial.println(speed);
        int velocidadGanancia = (topSpeed + speed);
        int velocidadPerdida =  (topSpeed - speed);  
        if(velocidadGanancia > maxSpeed ) velocidadGanancia = maxSpeed;
        if(velocidadPerdida < minSpeed)

      if (position > setPoint)
      { Serial.println("caso doblar derecha");
        Serial.println("velocidad motor derecho: ");
        Serial.println(topSpeed - speed);
        Serial.println("velocidad motor izquierdo: ");
        Serial.println(topSpeed + speed);
        //motorDer->GoAvance(topSpeed - speed);
        //motorIzq->GoAvance(topSpeed + speed);
      }
      if (position < setPoint)
      {
        Serial.println("caso doblar izquierda");
        Serial.println("velocidad motor derecho: ");
        Serial.println(topSpeed + speed);
        Serial.println("velocidad motor izquierdo: ");
        Serial.println(topSpeed - speed);
       // motorDer->GoAvance(topSpeed + speed);
        //motorIzq->GoAvance(topSpeed - speed);
      }

      last = proporcional;
    //      Serial.println("last: ");
      //  Serial.println(last);
        delay(1000);
    }
  }

  /*if (DEGUB)
  {
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println(position);
    }

    delay(350);
  }
  delay(1000);*/
  /*position = qtr.readLineWhite(sensorValues);
  Serial.println("posision : ");
  Serial.println(position);
  Serial.println("proporcional");
  Serial.println(position - setPoint);
  motorDer->GoAvance(30);
      motorIzq->GoAvance(30);*/
}
