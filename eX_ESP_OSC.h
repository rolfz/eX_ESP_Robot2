/*
 * 
 * 
 *  07.12.2016 RZ changes push to include emergency stop
 */
#ifndef EX_ESP_OSC_H
#define EX_ESP_OSC_H

#include "eX_ESP_WiFi.h"

float    fadder[4];
int      page=1, push[3], toggle[2]; 
boolean  ChangePage, RequestBAT, AutoMode;


float OSC_Extract_Param(int msgSize)
{
  u.Buff[0] = (unsigned char)PacketBuffer[msgSize-1];
  u.Buff[1] = (unsigned char)PacketBuffer[msgSize-2];
  u.Buff[2] = (unsigned char)PacketBuffer[msgSize-3];
  u.Buff[3] = (unsigned char)PacketBuffer[msgSize-4];
  return(u.d);
}

boolean OSC_MSG_Read()
{
  if (!WiFi_MSG_Read()) return false;

  if (PacketBuffer[1]=='p')  // команда PING
  {
#ifdef DEBUG_OSC
    Serial.println("rec. pint = request bat ");
#endif
    RequestBAT = true;
    return false;
  }
  if (PacketBuffer[0]=='/') page = int(PacketBuffer[1]) - 48;
  if (!(PacketBuffer[2]=='/')) 
  {
    ChangePage = true;
    return true;
  }
  if (PacketBuffer[3]=='f')  // команда FADER
  {
//    Serial.print("rec. fader ");
    fadder[int(PacketBuffer[8] - 49)] = OSC_Extract_Param(PacketSize);
//    Serial.println(OSC_Extract_Param(PacketSize));
    return true;
  }
  if (PacketBuffer[3]=='t')  // команда TOGGLE
  {
//    Serial.print("rec. toggle ");
    toggle[int(PacketBuffer[9] - 49)] = int(OSC_Extract_Param(PacketSize));
//    Serial.println(int(OSC_Extract_Param(PacketSize)));
  return true;
  }
  if (PacketBuffer[3]=='p')  // команда PUSH
  {
//    Serial.print("rec. push ");
    push[int(PacketBuffer[7] - 49)] = int(OSC_Extract_Param(PacketSize));
//    Serial.println(int(OSC_Extract_Param(PacketSize)));
    return true;
  } 
  return false;
}

#endif //EX_ESP_OSC_H
