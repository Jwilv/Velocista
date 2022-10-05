#include <Arduino.h>
#include <Motor.h>
#include <QTRSensors.h>

#define DEGUB 1

#define BUTTON 0

#define MOTOR_DIR_DER
#define MOTOR_PWM_DER

#define MOTOR_DIR_IZQ
#define MOTOR_PWM_IZQ

#define LED_BUILTIN 13

int topSpeed = 130;

float speed;

int p = 0;
int d = 0;
int i = 0;

int kp = 0;
int kd = 0;
int ki = 0;

int last = 0;

bool btnPress;

uint16_t position;

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
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //

  // print the calibration minimum values measured when emitters were on
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
}

void PositionAndValues()
{

  position = qtr.readLineWhite(sensorValues);

  /* print the sensor values as numbers from 0 to 1000, where 0 means maximum
   reflectance and 1000 means minimum reflectance, followed by the line position*/
  if (DEGUB)
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
}

void setup()
{ // configure the sensors
  Calibration();
  // constructors
  // MOTOR *motorDer = new MOTOR(MOTOR_DIR_DER,MOTOR_PWM_DER);
  // MOTOR *motorDer = new MOTOR(MOTOR_DIR_IZQ,MOTOR_PWM_IZQ);
  delay(1000);
}

void loop()
{
  btnPress = digitalRead(BUTTON) == 1;
  if (btnPress)
  {
    while (true)
    {
      PositionAndValues();
    }
  }
}
