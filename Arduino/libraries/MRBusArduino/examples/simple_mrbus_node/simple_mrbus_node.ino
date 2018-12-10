#include <MRBusArduino.h>
#include <EEPROM.h>
MRBus mrbus;

unsigned long lastUpdateTime, currentTime, lastTransmitTime;
unsigned long updateInterval = 2000;

void setup()
{
  uint8_t address = EEPROM.read(MRBUS_EE_DEVICE_ADDR);
  mrbus.begin();
  if (0xFF == address)
  {
    EEPROM.write(MRBUS_EE_DEVICE_ADDR, 0x07);
    address = EEPROM.read(MRBUS_EE_DEVICE_ADDR);
  }

  // Read the update interval out of EEPROM
  // By MRBus convention, it's stored in tenths of seconds.  The Arduino counter operates in milliseconds
  updateInterval = (((unsigned int)EEPROM.read(MRBUS_EE_DEVICE_UPDATE_H))<<8) + EEPROM.read(MRBUS_EE_DEVICE_UPDATE_L);
  updateInterval *= 100;

  if (updateInterval > 2000)
    updateInterval = 2000;

  mrbus.setNodeAddress(address);
  lastUpdateTime = currentTime = millis();
  pinMode(13, OUTPUT);
}

void pktActions(MRBusPacket& mrbPkt)
{
  // Is it broadcast or for us?  Otherwise ignore
  if (0xFF != mrbPkt.pkt[MRBUS_PKT_DEST] && mrbus.getNodeAddress() != mrbPkt.pkt[MRBUS_PKT_DEST])
    return;

  if('A' == mrbPkt.pkt[MRBUS_PKT_TYPE])
  {
    // Ping
    MRBusPacket replyPkt;
    replyPkt.pkt[MRBUS_PKT_DEST] = mrbPkt.pkt[MRBUS_PKT_SRC];
    replyPkt.pkt[MRBUS_PKT_SRC] = mrbus.getNodeAddress();
    replyPkt.pkt[MRBUS_PKT_TYPE] = 'a';
    replyPkt.pkt[MRBUS_PKT_LEN] = 6;
    mrbus.queueTransmitPacket(replyPkt);
  }
  else if ('C' == mrbPkt.pkt[MRBUS_PKT_TYPE])
  {
    // Command Packet
    if (0 == mrbPkt.pkt[6])
      digitalWrite(13, LOW);
    else
      digitalWrite(13, HIGH);

    MRBusPacket replyPkt;
    replyPkt.pkt[MRBUS_PKT_DEST] = mrbPkt.pkt[MRBUS_PKT_SRC];
    replyPkt.pkt[MRBUS_PKT_SRC] = mrbus.getNodeAddress();
    replyPkt.pkt[MRBUS_PKT_TYPE] = 'c';
    replyPkt.pkt[MRBUS_PKT_LEN] = 7;
    replyPkt.pkt[6] = mrbPkt.pkt[6];
    mrbus.queueTransmitPacket(replyPkt);
  }
  else if ('R' == mrbPkt.pkt[MRBUS_PKT_TYPE] && mrbPkt.pkt[MRBUS_PKT_LEN] >= 7)
  {
    // EEPROM Read
    MRBusPacket replyPkt;
    replyPkt.pkt[MRBUS_PKT_DEST] = mrbPkt.pkt[MRBUS_PKT_SRC];
    replyPkt.pkt[MRBUS_PKT_SRC] = mrbus.getNodeAddress();
    replyPkt.pkt[MRBUS_PKT_TYPE] = 'r';
    replyPkt.pkt[MRBUS_PKT_LEN] = 8;
    replyPkt.pkt[6] = mrbPkt.pkt[6];
    replyPkt.pkt[7] = EEPROM.read(mrbPkt.pkt[6]);
    mrbus.queueTransmitPacket(replyPkt);
  }
  else if ('W' == mrbPkt.pkt[MRBUS_PKT_TYPE] && mrbPkt.pkt[MRBUS_PKT_LEN] >= 8)
  {
    // EEPROM Write
    EEPROM.write(mrbPkt.pkt[6], mrbPkt.pkt[7]);

    MRBusPacket replyPkt;
    replyPkt.pkt[MRBUS_PKT_DEST] = mrbPkt.pkt[MRBUS_PKT_SRC];
    replyPkt.pkt[MRBUS_PKT_SRC] = mrbus.getNodeAddress();
    replyPkt.pkt[MRBUS_PKT_TYPE] = 'w';
    replyPkt.pkt[MRBUS_PKT_LEN] = 8;
    replyPkt.pkt[6] = mrbPkt.pkt[6];
    replyPkt.pkt[7] = EEPROM.read(mrbPkt.pkt[6]);
    mrbus.queueTransmitPacket(replyPkt);
  }

}

void loop()
{
  currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval)
  {
    // More than 2 seconds have elapsed since we sent a status packet  
    MRBusPacket statusPkt;
    statusPkt.pkt[MRBUS_PKT_DEST] = 0xFF;
    statusPkt.pkt[MRBUS_PKT_SRC] = mrbus.getNodeAddress();
    statusPkt.pkt[MRBUS_PKT_TYPE] = 'S';
    statusPkt.pkt[MRBUS_PKT_LEN] = 7;
    statusPkt.pkt[6] = 0;
    statusPkt.pkt[6] |= (digitalRead(13))?0x01:0;
    mrbus.queueTransmitPacket(statusPkt);
    lastUpdateTime = currentTime;
  }

  // If we have packets, try parsing them
  if (mrbus.hasRxPackets())
  {
    MRBusPacket mrbPkt;
    mrbus.getReceivedPacket(mrbPkt);
    pktActions(mrbPkt);
  }

  // If there are packets to transmit and it's been more than 20mS since our last transmission attempt, try again
  if (mrbus.hasTxPackets() && ((lastTransmitTime - currentTime) > 20))
  {
     lastTransmitTime = currentTime;
     mrbus.transmit();
  }
}

