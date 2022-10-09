#ifndef Motor_h
#include <Arduino.h>

class MOTOR{
 private:
 int dir,pwm;
 public:

 MOTOR(int dir,int pwm);

 void GoAvance(int speed);

 void GoBack(int speed);

 void Still();
};

#endif