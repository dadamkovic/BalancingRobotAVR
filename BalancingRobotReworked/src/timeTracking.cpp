/**
 * \file timeTracking.cpp
 * \author Daniel Adamkovic
 * \date 22/1/2019
 * \brief Definitions of all functions handling ATmega timers
 */
#include "timeTracking.h"

/**
 * \brief Sets up Timer5 for normal count up mode.
 */
void clockInit(){
    TCCR5A &= ~(_BV(WGM50)|_BV(WGM51));
    TCCR5B &= ~(_BV(WGM52));                //normal mode
}

/**
 * \brief Starts Timer's 5 countdown, with period set to approximately 30ms.
 * \return void
 */
void clockStart(){
    TCCR5B |= _BV(CS51);
    TCCR5B &= ~(_BV(CS50)|_BV(CS52));      //prescaler 8, f ~ 30Hz
}

/**
 * \brief Transforms Timer's 5 contents into seconds.
 * \return Seconds since the timer was started or reset.
 */
float clockTime(){
    return ((float)TCNT5/TIMER5_SIZE)*TIMER5_PERIOD;
}

/**
 * \brief Resets Timer 5 contents.
 */
void clockReset(){
    TCNT5 = 0;
}

/**
 * \brief Stops Timer 5's operation.
 * \return void
 */
void clockShutdown(){
    TCCR5B &= ~(_BV(CS50)|_BV(CS52)|_BV(CS51));      //clock stopped
}


/*
 * *******************************************************************************************
 */

/**
 * \brief Sets up Timer4 to trigger interrupt every 50ms.
 * \return void
 */
void encoderClockInit(){
    TCCR4A &= ~(_BV(WGM40)|_BV(WGM41));
    TCCR4B &= ~(_BV(WGM42));                //normal mode
    OCR4B = 12500;                          //interrupt should happen every 50ms ==> OCR4B = (INETRRUPT_PERIOD/TIMER_PERIOD)*TIMER_SIZE
    //OCR4B = 25000;
    TIMSK4 |= _BV(OCIE4B);                  //enable compare interrupt on OCR4B
}

/**
 * \brief Starts Timer 4's counting.
 * \return void
 */
void encoderClockStart(){
    TCCR4B |= (_BV(CS41)|_BV(CS40));
    TCCR4B &= ~_BV(CS42);      //prescaler 64, f = 3,81Hz
}

/**
 * \brief Stops Timer 4's operation.
 * \return void
 */
void encoderClockStop(){
    TCCR4B &= ~(_BV(CS40)|_BV(CS42)|_BV(CS41));      //clock stopped
}



