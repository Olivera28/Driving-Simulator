#include <Servo.h>
//servo object
Servo ESC1;
Servo ESC2;
Servo ESC3;

int Speed1;
int Speed2;
int Speed3;

int Timer;

bool tmp = false;
void setup() {
  //The pulses will be between 1 millsecond and 2 milliseconds in length
  Serial.begin(9600);

  Speed1 = 90;
  Speed2 = 90;
  Speed3 = 90;

  Timer = 0;

  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  
  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  ESC3.attach(5, 1000, 2000);
  delay(2000);
  
}

void loop() {

  //Serial.println("n!");
  //Read value of potentiometer from analog input pin A0
  //Speed = analogRead(A0);

  //Create conversion scale
  //Voltage/number/degree
  
  //Speed = map(Speed, 0, 1023, 0, 180);
  if (tmp == false){
      Speed1++;
      if(Speed1 == 160)tmp=true;
  }
  else {
      Speed1--;
      if(Speed1 == 20)tmp=false;
  }
//---------------------------------
  if (tmp == false){
    Timer++;
    if (Timer == 500) {
      Speed2 = 100;
      tmp = true;
      Timer = 0;
    }
  }
  else {
    Timer++;
    if (Timer == 500) {
      Speed2 = 90;
      tmp = false;
      Timer = 0;
    }
  }

  Serial.println(Timer);
  //Sends value as a PWM signal to the electronic speed controller
  ESC1.write(Speed1);
  ESC2.write(Speed2);
  ESC3.write(Speed2);
  delay(20);
}
