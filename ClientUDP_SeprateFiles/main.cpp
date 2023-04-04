#include "UDP_function.h"
#include "Pedal_function.h"
#include "Motor_function.h"

//Rotary encoder pins 
#define ENC_A 1 //10
#define ENC_B 2 //11

//variables for steering wheel FFB
int turn_lr = 0;
int motorResist = 0;

//variables for the rotary encoder function
int counter = 158; 
int aState;
int aLastState;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  //Digital pins for the push buttons
  pinMode(1,INPUT_PULLDOWN);
  pinMode(2,INPUT_PULLDOWN);
  pinMode(3,INPUT_PULLDOWN);

  //Digital pins for the rotary encoder
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);

  //Digital pins for the DC motor control
  pinMode(pwmA,OUTPUT); 
  pinMode(STBY,OUTPUT); 
  pinMode(AIN1,OUTPUT); 
  pinMode(AIN2,OUTPUT);

  // Gas Pedal Potentiometer
  pinMode (GAS, INPUT);

  // Break Pedal Potentiometer
  pinMode (BRAKE, INPUT);

  //attachInterrupt(digitalPinToInterrupt(ENCA),readTurn,RISING);
  

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);  
    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to wifi");
  Serial.println("\nStarting connection to server...");
  
  // if you get a connection, report back via serial:
  Udp.begin(UDP_PORT);

  // Reads the initial state of the outputA
   aLastState = digitalRead(ENC_A);
  
}


void loop() {

  acceleration = analogRead(A1);
  brake = analogRead(A2);

  //building the packet with respect to real data
  setGear();
  setVelocity();

  packet.gear = currentGear;
  packet.vel = (map(velocity, 0.0, 100.0, 820.0, 600.0)/10.0);
  //Serial.println(packet.vel);
  //noInterrupts(); // disable interrupts temporarily while reading
  
  aState = digitalRead(ENC_A); // Reads the "current" state of the outputA
  if (aState != aLastState){

     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(ENC_B) != aState) { 
      counter ++;
    } else {
      counter --;
    }
     
    // User is turning the steering wheel to the right
    Serial.print("Position: ");
    Serial.println(counter);
    if(145 < counter < 175){
      //Serial.print("Not Turning");
      stopMotor();
    }
    if(counter > 158){ //Clockwise condition
      turn_lr = map(counter, 158, 240, 105, 40);
      Serial.print("Turning: ");
      Serial.println(turn_lr);
      //rotateCW();
    }
    if(counter < 158){ //CounterClockwise condition
      turn_lr = map(counter, 70, 158, 190, 105);
      Serial.print("Turning: ");
      Serial.println(turn_lr);
      //rotateCCW();
    }
    
    // Wrap around when wheel has tunred passed three times to the right
    if(counter >= 316) counter = 316;
    // Wrap around when wheel has tunred passed three times to the left
    if(counter <= 0) counter = 0;
    
  }
  packet.turn = turn_lr; 
  //interrupts(); // turn interrupts back on
  // Serial.print("Packet_turn: ");
  // Serial.println(packet.turn);
  //printPacketInfo();
  // Copys gear, velocity, and turn angle from struct into the packetBuffer
  memcpy(packetBuffer, &packet.gear, sizeof(packet.gear));
  memcpy(packetBuffer+sizeof(packet.gear), &packet.vel, sizeof(packet.vel));
  memcpy(packetBuffer+sizeof(packet.gear)+sizeof(packet.vel), &packet.turn, sizeof(packet.turn));
         
  //Opens a packet, writes to the buffer, and sends the packet
  Udp.beginPacket(remoteIP, UDP_PORT);
  Udp.write(packetBuffer, sizeof(packetBuffer));
  Udp.endPacket();

  aLastState = aState; // Updates the previous state of the outputA with the current state
}
