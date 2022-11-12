#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Motor.h>
#include <QTRSensors.h> //libreria que ayuda a reconocer el error

#define DEGUB 0

#define BTON 0

#define DEBUG_CALCULOS 0

#define BUTTON 12

#define MOTOR_DIR_DER 3
#define MOTOR_PWM_DER 2

#define MOTOR_DIR_IZQ 4
#define MOTOR_PWM_IZQ 5

#define LED_BUILTIN 13

bool inicio = true;

int equiparamiento = 0;

int velocidadPuntaDer = 70;

int velocidadPuntaIzq = velocidadPuntaDer - equiparamiento;

float pidIzq;
float pidDer;

int maxSpeed = 255;

int minSpeed = 0;

float speed;

float proporcional;
float derivativa;
float integral;

int setPoint = 2500;

float kp = 0.0;
float kd = 0;
float ki = 0;

float last = 0;

int arranque = 30;

bool btnPress;

int position;

QTRSensors qtr;

const uint8_t SensorCount = 8;
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

void Comunication()
{
    if (bt.available())
    {
        char in = bt.read();
        if (in == KPS0001)
            kp += 0.001;
        else if (in == KDS0001)
            kd += 0.001;
        else if (in == KIS0001)
            ki += 0.001;
        else if (in == KPR0001)
            kp -= 0.001;
        else if (in == KDR0001)
            kd -= 0.001;
        else if (in == KIR0001)
            ki -= 0.001;
        else if (in == KPS001)
            kp += 0.01;
        else if (in == KDS001)
            kd += 0.01;
        else if (in == KIS001)
            ki += 0.01;
        else if (in == KPR001)
            kp -= 0.01;
        else if (in == KDR001)
            kd -= 0.01;
        else if (in == KIR001)
            ki -= 0.01;
        else if (in == KPS01)
            kp += 0.1;
        else if (in == KDS01)
            kd += 0.1;
        else if (in == KIS01)
            ki += 0.1;
        else if (in == KPR01)
            kp -= 0.1;
        else if (in == KDR01)
            kd -= 0.1;
        else if (in == KIR01)
            ki -= 0.1;
    }
}

/////////////////////////////

// calibration

void Calibration()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A9,A8,A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint16_t i = 0; i < 2000; i++)
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

    if (DEBUG_CALCULOS)
    {
        Serial.begin(9600);
        Serial.println("calibrando");
    }
    Calibration();
    if (BTON)
        bt.begin(9600);
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
    Serial.println("end setup");
}

void loop()
{
    btnPress = digitalRead(BUTTON) == 0;
    int position = qtr.readLineWhite(sensorValues);
    if (BTON)
        Comunication();
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
    // delay(250);

    if (btnPress)
    {
        motorDer->GoAvance(arranque + equiparamiento);
        motorIzq->GoAvance(arranque);

        while (true)
        {
            if (BTON)
                Comunication();
            inicio = false;
            position = qtr.readLineWhite(sensorValues);
            if (DEBUG_CALCULOS)
            {
                Serial.print("posision : ");
                Serial.println(position);
            }
            proporcional = (position) - (setPoint);

            if (DEBUG_CALCULOS)
            {
                Serial.print("proporcional : ");
                Serial.println(proporcional);
            }

            derivativa = (proporcional - last);

            // integral = (integral + proporcional);

            float speed = (proporcional * kp) + (derivativa * kd) + (integral * ki);

            last = proporcional;

            if (DEBUG_CALCULOS)
            {
                Serial.print("speed : ");
                Serial.println(speed);
            }
            pidIzq = (velocidadPuntaIzq + speed);
            pidDer = (velocidadPuntaDer - speed);

            if (pidIzq > maxSpeed)
                pidIzq = maxSpeed;
            else if (pidIzq < minSpeed)
                pidIzq = minSpeed;

            if (pidDer < minSpeed)
                pidDer = minSpeed;
            else if (pidDer > maxSpeed)
                pidDer = maxSpeed;
        }

        if (DEBUG_CALCULOS)
        {
            Serial.print("velocidad motor derecho: ");
            Serial.println(pidDer);
            Serial.print("velocidad motor izquierdo: ");
            Serial.println(pidIzq);
        }

        ////////////////////////////////
        motorDer->GoAvance(pidDer);
        motorIzq->GoAvance(pidIzq);
        ////////////////////////////////

        if (BTON)
        {
            bt.println("valor kp");
            bt.println(kp);
            bt.println("valor kd");
            bt.println(kd);
            bt.println("valor ki");
            bt.println(ki);
        }
        if (DEBUG_CALCULOS)
            delay(2000);
    }
}
