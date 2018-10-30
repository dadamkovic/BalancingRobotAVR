/*
 * encoders.cpp
 *
 *  Created on: Oct 20, 2018
 *      Author: daniel
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include "timeTracking.h"


extern volatile int16_t encoderAB, encoderCD;
volatile extern float speedAB,speedCD;


/*GPIO - initialization routines for interrupt handling
 * pins used are ==> (PE4, PG5), (PE5,PE3)
 */
void encodersInit(){
    DDRG &= ~(_BV(PG5));
    DDRE &= ~(_BV(PE4)| _BV(PE5) | _BV(PE3));

    EICRB |= (_BV(ISC40) | _BV(ISC41));         //PE4 as interrupt on rising edge
    EICRB |= (_BV(ISC50) | _BV(ISC51));         //PE5 as interrupt on rising edge

    EIMSK |=  _BV(INT4) | _BV(INT5);            //enables interrupts on PE4, PE5

    encoderClockInit();
    encoderClockStart();
};




ISR(INT4_vect){
    if( PING & _BV(PG5))encoderAB++;
    else encoderAB--;
};


/*
 ******************************************
 */

ISR(INT5_vect){
    if(PINE & _BV(PE3))encoderCD++;
    else encoderCD--;
}


/*
 * TODO : finishing measurement of time, manage to ease chip
 *
 */



ISR(TIMER4_COMPB_vect){
    speedAB = (encoderAB*0.0168)/0.05;      //374 tickov na 2pi ==> 2pi/374 = 0.0168
    speedCD = (encoderAB*0.0168)/0.05;      //casova konstanta pre meranie je dt = 0.05s
    encoderAB = 0;
    //encoderAB = 0;
    TCNT4 = 0;
}

