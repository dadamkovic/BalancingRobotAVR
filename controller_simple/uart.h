#ifndef UART_H
#define UART_H

#include "inttypes.h"
#define BAUD 9600       ///change for different baudrate
#include <util/setbaud.h>
#include <avr/io.h>
#include <util/delay.h>
#define UMSEL0 6
#define UMSEL1 7

void initUART();
void sendChar(uint8_t);
void sendString(const char[]);
char receiveChar();
void receiveString(char*);
bool avaliableSerial();




#endif // UART_H

