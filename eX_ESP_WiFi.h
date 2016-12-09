#ifndef EX_ESP_WIFI_H
#define EX_ESP_WIFI_H

#include <WiFiUdp.h>

#define  USE_UART

WiFiUDP UdP;

#include "accessData.h" // THIS FILE CONTAINS THE SSID AND PASSWORD FOR THE WIFI CONNECTION
uint16_t   RxPort = 8888;
uint16_t   TxPort = 9999;

char PacketBuffer[128];                 //tampon pour stocker les paquets reçus et envoyés
int  PacketSize = 0;

union{
  unsigned char Buff[4];
  float d;
}u;

void WiFi_Start()
{

 #ifdef SOFTAP
  Serial.println("\nStarting WIFI in SOFTAP mode");
  WiFi.softAP(ssid); // password can be added as 2nd parameter
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  #else
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF); 
  
  Serial.println("Starting WIFI in STATION mode");
  WiFi.mode(WIFI_STA);
  delay(1000);
    
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
  #endif
  UdP.begin(RxPort);
}

boolean WiFi_MSG_Read()
{
  PacketSize = UdP.parsePacket();
  if (!PacketSize) return false;
  UdP.read(PacketBuffer,PacketSize); // read the packet into the buffer
  return true;
}

void WiFi_MSG_Send_Float(char *c, int msgSize, float p)
{
  u.d = p;
  c[msgSize-4] = u.Buff[3];
  c[msgSize-3] = u.Buff[2];
  c[msgSize-2] = u.Buff[1];
  c[msgSize-1] = u.Buff[0];
  UdP.beginPacket(UdP.remoteIP(), TxPort);
  UdP.write(c, msgSize);
  UdP.endPacket();
}

#endif //EX_ESP_WIFI_H
