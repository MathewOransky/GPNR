/*************************************************************************
Title:    MRBus Arduino Library Header
Authors:  Nathan Holmes <maverick@drgw.net>, Colorado, USA
          Michael Petersen <railfan@drgw.net>, Colorado, USA
          Michael Prader, South Tyrol, Italy
File:     MRBusArduino.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2015 Nathan Holmes, Michael Petersen, and Michael Prader
    
    The MRBus library provides a way to easily interface Arduino applications
    with MRBus-based networks.  While written to be used with the Iowa Scaled
    Engineering mrb-ard shield, it should be easily compatible with any RS485
    driver hooked up to an Arduino serial port appropriately.
    
    The latest source can be obtained from ISE's Github repository here:
    https://github.com/IowaScaledEngineering/mrb-ard

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/

#ifndef MRBUS_AVR_H
#define MRBUS_AVR_H

#include <stdlib.h>
#include <avr/io.h>

// Size definitions
#define MRBUS_BUFFER_SIZE  0x14

// Packet component defines
#define MRBUS_PKT_DEST  0
#define MRBUS_PKT_SRC   1
#define MRBUS_PKT_LEN   2
#define MRBUS_PKT_CRC_L 3
#define MRBUS_PKT_CRC_H 4
#define MRBUS_PKT_TYPE  5
#define MRBUS_PKT_SUBTYPE 6

// mrbus_status masks
#define MRBUS_RX_PKT_READY    0x01
#define MRBUS_RX_PKT_OVERFLOW 0x02
#define MRBUS_TX_BUF_ACTIVE   0x40
#define MRBUS_TX_PKT_READY    0x80

// mrbus_activity states
#define MRBUS_ACTIVITY_IDLE          0
#define MRBUS_ACTIVITY_RX            1
#define MRBUS_ACTIVITY_RX_COMPLETE   2

// Specification-defined EEPROM Addresses
#define MRBUS_EE_DEVICE_ADDR         0
#define MRBUS_EE_DEVICE_OPT_FLAGS    1
#define MRBUS_EE_DEVICE_UPDATE_H     2
#define MRBUS_EE_DEVICE_UPDATE_L     3

// Version flags
#define MRBUS_VERSION_WIRELESS 0x80
#define MRBUS_VERSION_WIRED    0x00

#define MRBUS_BAUD   57600

// AVR type-specific stuff
// Define the UART port and registers used for XBee communication
// Follows the format of the AVR UART library by Fleury/Sharpe

#if defined(__AVR_ATmega162__)

#define MRBUS_ATMEGA_USART0_SIMPLE
#define MRBUS_UART_RX_INTERRUPT    USART0_RXC_vect
#define MRBUS_UART_TX_INTERRUPT    USART0_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART0_TXC_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR            UBRR0L
#define MRBUS_UART_SCR_A           UCSR0A
#define MRBUS_UART_SCR_B           UCSR0B
#define MRBUS_UART_SCR_C           UCSR0C
#define MRBUS_UART_DATA            UDR0
#define MRBUS_UART_UDRIE           UDRIE0
#define MRBUS_RXEN                 RXEN0
#define MRBUS_TXEN                 TXEN0
#define MRBUS_RXCIE                RXCIE0
#define MRBUS_TXCIE                TXCIE0
#define MRBUS_RX_ERR_MASK          (_BV(FE0) | _BV(DOR0))



#elif  defined(__AVR_ATmega8__)

#define MRBUS_ATMEGA_USART_SIMPLE
#define MRBUS_UART_RX_INTERRUPT    USART_RXC_vect
#define MRBUS_UART_TX_INTERRUPT    USART_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART_TXC_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR            UBRRL
#define MRBUS_UART_SCR_A           UCSRA
#define MRBUS_UART_SCR_B           UCSRB
#define MRBUS_UART_SCR_C           UCSRC
#define MRBUS_UART_DATA            UDR
#define MRBUS_UART_UDRIE           UDRIE
#define MRBUS_RXEN                 RXEN
#define MRBUS_TXEN                 TXEN
#define MRBUS_RXCIE                RXCIE
#define MRBUS_TXCIE                TXCIE
#define MRBUS_RX_ERR_MASK          (_BV(FE) | _BV(DOR))



#elif defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
    defined(__AVR_ATmega328P__) 
#define MRBUS_ATMEGA_USART
#define MRBUS_UART_RX_INTERRUPT    USART_RX_vect
#define MRBUS_UART_TX_INTERRUPT    USART_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART_TX_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR            UBRR0
#define MRBUS_UART_SCR_A           UCSR0A
#define MRBUS_UART_SCR_B           UCSR0B
#define MRBUS_UART_SCR_C           UCSR0C
#define MRBUS_UART_DATA            UDR0
#define MRBUS_UART_UDRIE           UDRIE0
#define MRBUS_RXEN                 RXEN0
#define MRBUS_TXEN                 TXEN0
#define MRBUS_RXCIE                RXCIE0
#define MRBUS_TXCIE                TXCIE0
#define MRBUS_RX_ERR_MASK          (_BV(FE0) | _BV(DOR0))

#elif defined(__AVR_ATmega32U4__)

#define MRBUS_ATMEGA_USART1
#define MRBUS_UART_RX_INTERRUPT    USART1_RX_vect
#define MRBUS_UART_TX_INTERRUPT    USART1_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART1_TX_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  1       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   3       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   2       /* PD0 */
#endif


#define MRBUS_UART_UBRR           UBRR1
#define MRBUS_UART_SCR_A          UCSR1A
#define MRBUS_UART_SCR_B          UCSR1B
#define MRBUS_UART_SCR_C          UCSR1C
#define MRBUS_UART_DATA           UDR1
#define MRBUS_UART_UDRIE          UDRIE1
#define MRBUS_RXEN                RXEN1
#define MRBUS_TXEN                TXEN1
#define MRBUS_RXCIE               RXCIE1
#define MRBUS_TXCIE               TXCIE1
#define MRBUS_RX_ERR_MASK         (_BV(FE1) | _BV(DOR1))


#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)

#define MRBUS_ATMEGA_USART0
#define MRBUS_UART_RX_INTERRUPT    USART0_RX_vect
#define MRBUS_UART_TX_INTERRUPT    USART0_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT  USART0_TX_vect
#define MRBUS_PORT                 PORTD
#define MRBUS_PIN                  PIND
#define MRBUS_DDR                  DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                  4       /* PD4 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PD0 */
#endif

#define MRBUS_UART_UBRR           UBRR0
#define MRBUS_UART_SCR_A          UCSR0A
#define MRBUS_UART_SCR_B          UCSR0B
#define MRBUS_UART_SCR_C          UCSR0C
#define MRBUS_UART_DATA           UDR0
#define MRBUS_UART_UDRIE          UDRIE0
#define MRBUS_RXEN                RXEN0
#define MRBUS_TXEN                TXEN0
#define MRBUS_RXCIE               RXCIE0
#define MRBUS_TXCIE               TXCIE0
#define MRBUS_RX_ERR_MASK         (_BV(FE0) | _BV(DOR0))

#elif defined(__AVR_ATmega2560__)
/* ATmega with four USART, use the first one */
#define MRBUS_ATMEGA_USART0
#define MRBUS_UART_RX_INTERRUPT   USART0_RX_vect
#define MRBUS_UART_TX_INTERRUPT   USART0_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT USART0_TX_vect
#define MRBUS_PORT                PORTE
#define MRBUS_PIN                 PINE
#define MRBUS_DDR                 DDRE

#ifndef MRBUS_TXE
#define MRBUS_TXE                  4       /* PE4 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                   1       /* PE1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                   0       /* PE0 */
#endif

#define MRBUS_UART_UBRR           UBRR0     /*Guess*/
#define MRBUS_UART_SCR_A          UCSR0A
#define MRBUS_UART_SCR_B          UCSR0B
#define MRBUS_UART_SCR_C          UCSR0C    /*Guess*/
#define MRBUS_UART_DATA           UDR0
#define MRBUS_UART_UDRIE          UDRIE0
#define MRBUS_RXEN                RXEN0     /*Guess*/
#define MRBUS_TXEN                TXEN0     /*Guess*/
#define MRBUS_RXCIE               RXCIE0    /*Guess*/
#define MRBUS_TXCIE               TXCIE0    /*Guess*/
#define MRBUS_RX_ERR_MASK         (_BV(FE0) | _BV(DOR0)) /*Guess*/


#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)

#define MRBUS_ATTINY_USART
#define MRBUS_UART_RX_INTERRUPT   USART_RX_vect
#define MRBUS_UART_TX_INTERRUPT   USART_UDRE_vect
#define MRBUS_UART_DONE_INTERRUPT USART_TX_vect
#define MRBUS_PORT                PORTD
#define MRBUS_PIN                 PIND
#define MRBUS_DDR                 DDRD

#ifndef MRBUS_TXE
#define MRBUS_TXE                 2       /* PD2 */
#endif
#ifndef MRBUS_TX
#define MRBUS_TX                  1       /* PD1 */
#endif
#ifndef MRBUS_RX
#define MRBUS_RX                  0       /* PD0 */
#endif

#define MRBUS_UART_UBRRH          UBRRH
#define MRBUS_UART_UBRRL          UBRRL
#define MRBUS_UART_SCR_A          UCSRA
#define MRBUS_UART_SCR_B          UCSRB
#define MRBUS_UART_SCR_C          UCSRC
#define MRBUS_UART_DATA           UDR
#define MRBUS_UART_UDRIE          UDRIE
#define MRBUS_RXEN                RXEN
#define MRBUS_TXEN                TXEN
#define MRBUS_RXCIE               RXCIE
#define MRBUS_TXCIE               TXCIE
#define MRBUS_RX_ERR_MASK         (_BV(FE) | _BV(DOR))
#else
#error "No UART definition for MCU available"
#error "Please feel free to add one and send us the patch"
#endif

typedef struct 
{
	uint8_t pkt[MRBUS_BUFFER_SIZE];
} MRBusPacket;

class MRBus
{
	public:
		void begin();
		uint8_t hasRxPackets();
		uint8_t hasTxPackets();
		bool queueTransmitPacket(uint8_t* pkt, uint8_t len);
		bool queueTransmitPacket(MRBusPacket &mrbPkt);
		bool getReceivedPacket(uint8_t* pkt, uint8_t len);			
		bool getReceivedPacket(MRBusPacket &mrbPkt);
		uint8_t transmit();
		uint8_t transmitBlocking();
		bool isTransmitting();
		void setNodeAddress(uint8_t nodeAddress);
		uint8_t getNodeAddress();
		void setNodePriority(uint8_t nodePriority);
		uint8_t getNodePriority();
		bool doCommonPacketHandlers(const MRBusPacket &mrbPkt);

	private:
		uint8_t nodeAddress;
		uint8_t nodePriority;
		uint8_t nodeLoneliness;
		uint16_t crc16Update(uint16_t crc, uint8_t a);
		uint8_t mrbusArbBitSend(uint8_t bitval);
};


#endif // MRBUS_AVR_H


