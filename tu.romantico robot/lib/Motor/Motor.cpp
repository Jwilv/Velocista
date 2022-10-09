#include <Motor.h>
#include <QTRSensors.h>
#include <Arduino.h>

MOTOR::MOTOR(int dir,int pwm){
this->dir = dir;
this->pwm = pwm;

pinMode(this->dir,OUTPUT);
pinMode(this->pwm,OUTPUT);

}

void MOTOR::GoAvance(int speed){

digitalWrite(dir,LOW);
analogWrite(pwm,speed);

}

void MOTOR::GoBack(int speed){

digitalWrite(dir,HIGH);
analogWrite(pwm,speed);

}

void MOTOR::Still(){
    analogWrite(pwm, 0 );
}