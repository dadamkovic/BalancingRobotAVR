/*
 * uart.h
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */
/*
 * It is necessary to enable interupts in main() through sei()
 */

#ifndef UART_H
#define UART_H


#define U2X 0
#define UMSEL0 6
#define UMSEL1 7
#define BAUD 9600       ///change for different baudrate
#define USE_2X 0
#define USE_INTERRUPT 1

#include <util/setbaud.h>
#include "inttypes.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

extern volatile char direction[3];
extern volatile uint8_t tracking;
extern volatile uint8_t directionReceived;


void initUART();
void sendChar(uint8_t);
void sendString(const char[]);
char receiveChar();
void receiveString(char*);
bool avaliableSerial();




#endif // UART_H


