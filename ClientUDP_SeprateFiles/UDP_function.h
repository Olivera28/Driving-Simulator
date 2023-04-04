#ifndef UDP_FUNCTION_H
#define UDP_FUNCTION_H

#include <Arduino.h>
#include <SPI.h>          // needed for Arduino versions later than 0018
#include <WiFiNINA.h>
#include <WiFiUdp.h> 
//---------------------------->
// UDP
inline WiFiUDP Udp;
inline IPAddress remoteIP(10,144,113,62);

// Packet struct for sending gear, velocity (vel), and angle of turn (turn)
struct UdpPacket{
  char gear = 'p';
  double vel = 0.0;
  int turn = 0;
}inline packet;

// Creates packetBuffer to send with UDP Packet containing the struct
inline char packetBuffer[sizeof(packet)];

// Set WiFi credentials
#define WIFI_SSID ""
#define WIFI_PASS ""
#define UDP_PORT 2390

inline int status = WL_IDLE_STATUS;

void printPacketInfo();
void printWifiStatus();

#endif
