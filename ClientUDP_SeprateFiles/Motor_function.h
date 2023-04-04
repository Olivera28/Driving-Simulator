#ifndef MOTOR_FUNCTION_H
#define MOTOR_FUNCTION_H
#include <Arduino.h>

//------------------------------------------->
#define pwmA 0 
#define STBY 8 
#define AIN1 6 
#define AIN2 7 

void rotateCW();
void rotateCCW();
void stopMotor();


#endif
