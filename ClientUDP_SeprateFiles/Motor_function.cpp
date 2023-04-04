#include "Motor_function.h"

//------------------>
void rotateCW() {
  //motorResist = map(counter, 0,-237,0,237);
  digitalWrite(STBY,HIGH); 
  digitalWrite(AIN1,LOW); 
  digitalWrite(AIN2,HIGH);
  //Serial.println(motorResist); 
  analogWrite(pwmA,255); 
  delay(5);
  analogWrite(pwmA,0);
  digitalWrite(AIN2,LOW); 
}

void rotateCCW() {
  //motorResist = map(counter,0,238,0,-237);
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH); 
  digitalWrite(AIN2,LOW);
  
  analogWrite(pwmA,255); 
  delay(5);
  analogWrite(pwmA,0);
  digitalWrite(AIN1,LOW);
}

void stopMotor(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW); 
  digitalWrite(AIN2,LOW);
}
