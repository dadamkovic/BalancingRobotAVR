#ifndef INIT_FUNCTIONS_H
#define INIT_FUNCTIONS_H

#include "uart.h"
#define DOWN    0b00001000
#define UP      0b00000100
#define LEFT    0b00000010
#define RIGHT   0b00000001


void initBLUETOOTH();
void initPINS();
void sendCommand();


void initBLUETOOTH(){
    sendString("AT+ROLE0\r\n");
    //sendString("AT+ROLE0\n");
    _delay_ms(1000);
    sendString("AT+UUIDFFE0\r\n");
    //sendString("AT+UUID0xFFE0\n");
    _delay_ms(1000);
    sendString("AT+CHARFFE1\r\n");
    _delay_ms(1000);
    sendString("AT+NAMErobotController\r\n");
    _delay_ms(1000);

}



void initPINS(){
    MCUCR &= ~_BV(PUD);
    DDRB &= ~(0b00001111);
    PORTB |= 0b00001111;    //PULLUP resistors

}

void sendCommand(){
    if(!(PINB & UP))sendChar('U');
    if(!(PINB & DOWN))sendChar('D');
    if(!(PINB & LEFT))sendChar('L');
    if(!(PINB & RIGHT))sendChar('R');
    if(!(( PINB & 0b00001111) ^ 0b00001111))sendChar('H');      //masking 4 MBS in PINB and testing whether 4 LSB are zeroes
    sendChar('\n');
}

#endif
