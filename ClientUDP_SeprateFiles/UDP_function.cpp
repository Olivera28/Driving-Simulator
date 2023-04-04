#include "UDP_function.h"

//Function to print the status of the Wifi connection
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


// Prints the packets values as well as the encoded values
void printPacketInfo(){
  Serial.print("gear= ");
  Serial.println(packet.gear);
  Serial.print("vel= ");
  Serial.println(packet.vel);
  Serial.print("turn= ");
  Serial.println(packet.turn);
  //Serial.println(packetBuffer);
  //Serial.println(sizeof(packetBuffer));
  delay(100);
}
