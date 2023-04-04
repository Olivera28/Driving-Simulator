#ifndef PEDAL_FUNCTION_H
#define PEDAL_FUNCTION_H
#include <Arduino.h>

//------------------------------------------->
// Gas Pedal Pin
#define GAS A1
#define BRAKE A2

//  Gas Pedal Variables
const float MAX_SPEED = 100.0;
const float MIN_ACCELERATION = 0.5;

inline char currentGear = 'p';
//global variable to store the value of the acceleration pedal
inline double acceleration = 0.0;
inline double brake = 0.0;
inline double velocity = 0.0;

void setVelocity();
void setGear();


#endif
