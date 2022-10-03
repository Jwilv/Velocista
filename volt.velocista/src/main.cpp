#include <Arduino.h>
#include <Motor.h>
#include <QTRSensors.h>

#define MOTOR_DIR_DER
#define MOTOR_PWM_DER

#define MOTOR_DIR_IZQ
#define MOTOR_PWM_IZQ

#define LED_BUILTIN 13

QTRSensors qtr;

const uint8_t SensorCount = 8 ;
uint16_t sensorValues[SensorCount];

void setup()
{
 //for(){
  //map(sensores[i],0,1024,1024,0);
 //}
 map(A0,0,1024,1024,0);
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  //qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}

/*void setup() {
//MOTOR *motorDer = new MOTOR(MOTOR_DIR_DER,MOTOR_PWM_DER);
//MOTOR *motorDer = new MOTOR(MOTOR_DIR_IZQ,MOTOR_PWM_IZQ);
}

void loop() {
  // put your main code here, to run repeatedly:
}*/