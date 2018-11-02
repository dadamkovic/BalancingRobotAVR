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

#include <stdlib.h>

#define U2X 0
#define BAUD 9600       ///change for different baudrate
#define USE_INTERRUPT 1

#include <util/setbaud.h>
#include "inttypes.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

extern volatile char controllerData[3];
extern volatile uint8_t receiveCommandFlag;

void initControllerUART();
void initInterfaceUART();
void interfaceSendChar(uint8_t);
void interfaceSendString(const char[]);
char interfaceReceiveChar();
void interfaceReceiveString(char*);
bool interfaceAvaliableSerial();
void interfaceSendFloat(float);




#endif // UART_H

