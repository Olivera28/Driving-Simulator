#include "Pedal_function.h"
#include "UDP_function.h"

//Function to chage the gear with respect to three push buttons
void setGear(){
  if (digitalRead(3) == 1){
     currentGear = 'p';
   }
   if (digitalRead(4) == 1){
     currentGear = 'd';
   }
   if (digitalRead(5) == 1){
     currentGear = 'r';
   }
}

//function to set the velocity of our RC car
void setVelocity(){
  // Maps potentiometer to acceleration values
  //acceleration = map(acceleration, 0, 1023, 60, 82);
  acceleration = acceleration / 1000;
  brake = map(brake, 0, 1023, 0, 640);
  brake = brake / 10000;
  //Serial.println(brake);

  // Accelerates car past a specefic threshold up to a max speed
  if (acceleration > MIN_ACCELERATION && velocity <= MAX_SPEED){
    velocity += acceleration*.1;
  }

   // stops car at low enough velocity
  if (velocity < .03){
    velocity = 0.0;
  }

   // Decelerates if not accelerating
  if (acceleration < MIN_ACCELERATION & velocity > .02){
    velocity -= .05;
  }
  //velocity = map(velocity, 0.0, 100.0, 82.0, 60.0);
  if (packet.gear == 'r'){
    packet.vel = (map(velocity, 0.0, 100.0, 1018.0, 1300.0)/10.0);
  }
  else{
    packet.vel = (map(velocity, 0.0, 100.0, 820.0, 600.0)/10.0);
  }

  Serial.println(velocity);
}
