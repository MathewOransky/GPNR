/*************************************************************************
Title:    MRBus Simple Terminal Example Sketch
Authors:  Nathan D. Holmes <maverick@drgw.net>
License:  GNU General Public License v3

Description:
 
 This sketch is an example that will display any/all valid MRBus packets going over
 the wire. 

 ** IMPORTANT NOTE**:  This will almost certainly only work on Arduino Leonardo boards,
 since they use an emulated serial port over USB in addition to a real hardware serial port.

LICENSE:
    Copyright (C) 2016 Nathan D. Holmes (maverick@drgw.net)
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <MRBusArduino.h>
MRBus mrbus;

#define INTERFACE_ADDRESS 0xFE

void setup()
{
  Serial.begin(115200);
  mrbus.begin();
  mrbus.setNodeAddress(INTERFACE_ADDRESS);
  pinMode(13, OUTPUT);
}

void displayPacket(MRBusPacket &rxPkt)
{
  Serial.write("P:");
  
  for (byte i=0; i<rxPkt.pkt[MRBUS_PKT_LEN]; i++)
  {
    char hexByte[5];
    sprintf(hexByte, "%02X ", rxPkt.pkt[i]);
    Serial.write(hexByte);
  }
  Serial.write("\r\n");
}

void loop()
{
  // If we have packets, send them to the PC
  if (mrbus.hasRxPackets())
  {
    MRBusPacket mrbPkt;
    mrbus.getReceivedPacket(mrbPkt);
    displayPacket(mrbPkt);
  }
}

