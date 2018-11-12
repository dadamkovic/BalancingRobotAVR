/*
 * encoders.cpp
 *
 *  Created on: Oct 20, 2018
 *      Author: daniel
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include "timeTracking.h"
#include "motorControl.h"

extern MotorDrive motors;
//extern volatile int16_t encoderAB, encoderCD;
//volatile extern float speedAB,speedCD;


/*GPIO - initialization routines for interrupt handling
 * pins used are ==> (PE4, PA0), (PE5,PA2)
 */
void encodersInit(){
    DDRA &= ~(_BV(PA0) | _BV(PA2));
    DDRE &= ~(_BV(PE4)| _BV(PE5));

    EICRB |= (_BV(ISC40) | _BV(ISC41));         //PE4 as interrupt on rising edge
    EICRB |= (_BV(ISC50) | _BV(ISC51));         //PE5 as interrupt on rising edge

    EIMSK |=  _BV(INT4) | _BV(INT5);            //enables interrupts on PE4, PE5

    encoderClockInit();
    encoderClockStart();
};




ISR(INT4_vect){
    if( PINA & _BV(PA0))motors.encoderAB++;
    else motors.encoderAB--;
};


/*
 ******************************************
 */

ISR(INT5_vect){
    if(PINA & _BV(PA2))motors.encoderCD++;
    else motors.encoderCD--;
}


/*
 * TODO : finishing measurement of time, manage to ease chip
 *
 */



ISR(TIMER4_COMPB_vect){
    motors.speedAB = (motors.encoderAB*0.0168)/0.05;      //374 tickov na 2pi ==> 2pi/374 = 0.0168
    motors.speedCD = (motors.encoderAB*0.0168)/0.05;      //casova konstanta pre meranie je dt = 0.05s
    motors.encoderAB = 0;
    motors.encoderCD = 0;
    TCNT4 = 0;
}

