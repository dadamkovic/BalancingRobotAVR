#include "timeTracking.h"
/*
 * function : sets up Timer5 for normal operation with reasonable period value for tracking milliseconds
 */
void clockInit(){
    TCCR5A &= ~(_BV(WGM50)|_BV(WGM51));
    TCCR5B &= ~(_BV(WGM52));                //normal mode
}

void clockStart(){
    TCCR5B |= _BV(CS51);
    TCCR5B &= ~(_BV(CS50)|_BV(CS52));      //prescaler 8, f = 30Hz
}

float clockTime(){
    return (TCNT5/65535)*0.032768;        //calculates time based on the amount is TCNT5 already accumulated
}

void clockReset(){
    TCNT5 = 0;
}

void clockShutdown(){
    TCCR5B &= ~(_BV(CS50)|_BV(CS52)|_BV(CS51));      //clock stopped
}


/*
 * *******************************************************************************************
 */

void encoderClockInit(){
    TCCR4A &= ~(_BV(WGM40)|_BV(WGM41));
    TCCR4B &= ~(_BV(WGM42));                //normal mode
    OCR4B = 12500;                          //interrupt should happen every 50ms ==> OCR4B = (0.05/0.262144)*65536
    TIMSK4 |= _BV(OCIE4B);                  //enable compare interrupt on OCR4B
}

void encoderClockStart(){
    TCCR4B |= (_BV(CS41)|_BV(CS40));
    TCCR4B &= ~_BV(CS42);      //prescaler 64, f = 3,81Hz
}

void encoderClockStop(){
    TCCR4B &= ~(_BV(CS40)|_BV(CS42)|_BV(CS41));      //clock stopped
}
