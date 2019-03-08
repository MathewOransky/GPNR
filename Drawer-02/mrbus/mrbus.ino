#include <MRBusArduino.h>

#include <SPI.h>
#include <EEPROM.h>
MRBus mrbus;

unsigned long lastUpdateTime, currentTime=0, lastTransmitTime;
unsigned long updateInterval = 2000;
const int slaveSelectPin = 10;

void setup()
{
/*  uint8_t address = EEPROM.read(MRBUS_EE_DEVICE_ADDR);
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
*/
  if (updateInterval > 2000)
    updateInterval = 2000;

//  mrbus.setNodeAddress(address);
  lastUpdateTime = currentTime = millis();
  pinMode(3, OUTPUT);
  pinMode(slaveSelectPin, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(53, INPUT_PULLUP); //3way A
  pinMode(52, INPUT_PULLUP); //3way B
  pinMode(51, INPUT_PULLUP); //3way C
  pinMode(50, INPUT_PULLUP); //Switch 4
  pinMode(49, INPUT_PULLUP); 
  pinMode(48, INPUT_PULLUP);
  pinMode(47, INPUT_PULLUP);
  pinMode(46, INPUT_PULLUP);
  pinMode(45, INPUT_PULLUP);
  pinMode(44, INPUT_PULLUP);
  pinMode(43, INPUT_PULLUP);
  pinMode(42, INPUT_PULLUP);
  pinMode(41, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP);
  pinMode(39, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(24, OUTPUT); //LED for Switch 4
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);
 
//  Serial.begin(9600);
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
      digitalWrite(3, LOW); 
    else
      digitalWrite(3, HIGH); 

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
  else if ('V' == mrbPkt.pkt[MRBUS_PKT_TYPE])
  {
    // Version Packet
    MRBusPacket replyPkt;
    replyPkt.pkt[MRBUS_PKT_DEST] = mrbPkt.pkt[MRBUS_PKT_SRC];
    replyPkt.pkt[MRBUS_PKT_SRC] = mrbus.getNodeAddress();
    replyPkt.pkt[MRBUS_PKT_TYPE] = 'v';
    replyPkt.pkt[MRBUS_PKT_LEN] = 15;
    replyPkt.pkt[6] = 0;
    replyPkt.pkt[7] = 0;
    replyPkt.pkt[8] = 0;
    replyPkt.pkt[9] = 3;
    replyPkt.pkt[10] = 0;
    replyPkt.pkt[11] = 0;
    replyPkt.pkt[12] = 'A';
    replyPkt.pkt[13] = 'R';
    replyPkt.pkt[14] = 'D';
    mrbus.queueTransmitPacket(replyPkt);
  }
}

void loop()
{
  int val;

  currentTime = millis();
/*  if (currentTime - lastUpdateTime >= updateInterval)
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

    digitalWrite(slaveSelectPin,LOW);
    //Decimal point
    SPI.transfer(0x77);
    SPI.transfer(2);
    digitalWrite(slaveSelectPin,HIGH);
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
*/
  //Turnout 1,2,3
  //Only one of the three inputs should be low
  if(0 == digitalRead(53)){
    digitalWrite(28, 0);
    digitalWrite(27, 1);
    digitalWrite(26, 0);
    digitalWrite(25, 1);
    
    digitalWrite(39, 1);
    digitalWrite(38, 0);
    digitalWrite(37, 1);
  }else if( 0 == digitalRead(52)){
    digitalWrite(28, 1);
    digitalWrite(27, 0);
    digitalWrite(26, 0);
    digitalWrite(25, 1);

    digitalWrite(39, 0); //Correct
    digitalWrite(38, 0); //X
    digitalWrite(37, 1); //Correct
  }else if (0 == digitalRead(51)){
    digitalWrite(28, 0);
    digitalWrite(27, 0);
    digitalWrite(26, 1);
    digitalWrite(25, 0);
    
    digitalWrite(39, 1);
    digitalWrite(38, 1);
    digitalWrite(37, 0);
  }else{
    digitalWrite(28, 0);
    digitalWrite(27, 0);
    digitalWrite(26, 0);
    digitalWrite(25, 0);
    
    digitalWrite(39, 0);
    digitalWrite(38, 0);
    digitalWrite(37, 0);
  }

  //Turnout 4
  val = digitalRead(50);
  digitalWrite(24, !val);
  digitalWrite(36, !val);

  //Turnout 5
  val = digitalRead(49);
  digitalWrite(23, !val);
  digitalWrite(35, !val);

  //Turnout 6/7 Crossover
  val = digitalRead(48);
  digitalWrite(22, val);  
  digitalWrite(34, !val);
  digitalWrite(33, !val);

  //Turnout 8
  val = digitalRead(47);
  digitalWrite(21, !val);  
  digitalWrite(32, !val);

  //Turnout 9
  val = digitalRead(46);
  digitalWrite(20, val);  
  digitalWrite(31, val);

  //Turnout 10
  val = digitalRead(45);
  digitalWrite(19, !val);  
  digitalWrite(30, !val);

  //Turnout 11
  val = digitalRead(44);
  digitalWrite(18, !val);  
  digitalWrite(29, !val);

  //Turnout Other
  val = digitalRead(43);
  digitalWrite(17, !val);  
  //Motor is remote

  //Turnout Other 2, 3, 4
  //Only one of the three inputs should be low
  if(0 == digitalRead(42)){
    digitalWrite(16, 0);
    digitalWrite(15, 0);
    digitalWrite(14, 1);
    //Motor is remote
  }else if( 0 == digitalRead(41)){
    digitalWrite(16, 0);
    digitalWrite(15, 1);
    digitalWrite(14, 0);
    //Motor is remote
  }else if (0 == digitalRead(40)){
    digitalWrite(16, 1);
    digitalWrite(15, 0);
    digitalWrite(14, 0);
    //Motor is remote  
  }else{
    digitalWrite(16, 0);
    digitalWrite(15, 1);
    digitalWrite(14, 0);
    //Motor is remote  
  }

  

  digitalWrite(LED_BUILTIN,(currentTime/128)%2);
}
