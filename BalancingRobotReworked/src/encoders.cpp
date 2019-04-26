/**
 * \file encoders.cpp
 * \author Daniel Adamkovic
 * \date 22/1/2019
 * \brief Functions and ISR routines necessary for correct reading of encoder values.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include "timeTracking.h"
#include "motorControl.h"
#include "uart.h"
#include "utility.h"

extern MotorControl motors;

FIFOBuffer bufferA;
FIFOBuffer bufferB;
/**
 * \brief Used to calculate by how big angle have the wheels turned in 50ms
 *
 *  The encoders detect approximately 374 rising edges per one turn. We know that angle of one turn,
 *  (full circle) is 2*PI. From that we can determine that the angle for every individual edge detected is
 *  2*PI/374 == 0,0168 radians per edge.
 */
#define ANGLE_PER_TICK 0.0168


/**
 * \brief Timer4 interrupt set up to occur every 50 ms.
 */
#define MOTOR_SAMPLE_TIME 0.025

/**
 * \brief Radius of the robot wheel.
 */
#define WHEEL_RADIUS 0.033

/**
 * \brief Determines the behavior of the motor's speed filter.
 *
 * This constant can be changed to any value between 0-1. The higher it is set the stronger the filtration is.
 *
 * \note It is not recommended to use vales close to the extremes as the filter then becomes useless.
 */
#define MOTOR_FILTER_CONSTANT 0.6
//#define MOTOR_FILTER_CONSTANT 0.25
/*GPIO - initialization routines for interrupt handling
 * pins used are ==> (PE4, PA0), (PE5,PA2)
 */
/**
 * \brief Initializes pins connected to the encoders and timers for time-tracking.
 * \return void
 * \note Called only once in the beginning of the program, before main loop.
 *
 * We decided to use pins in pairs (interrupt_enabled, normal_operation). Interrupt pins
 * detect the rising edge of the encoder signal and the code then checks if the signal
 * on the other pin is already high, this varies based on which direction the wheel moved.
 * Initialization functions are then called to start the timer, which afterwards controls the
 * interval in which the data from encoders is processed.
 */
void initEncoders(){
    DDRA &= ~(_BV(PA0) | _BV(PE5));             //pins as inputs
    DDRE &= ~(_BV(PE4)| _BV(PA2));

    EICRB |= (_BV(ISC40) | _BV(ISC41));         //PE4 as interrupt on rising edge
    EICRB |= (_BV(ISC50) | _BV(ISC51));         //PE5 as interrupt on rising edge

    EIMSK |=  _BV(INT4) | _BV(INT5);            //enables interrupts on PE4, PE5

    encoderClockInit();
    encoderClockStart();
};

/**
 * \see initEncoders
 */
ISR(INT4_vect){
    if( PINA & _BV(PA2))motors.encoderAB--;
    else motors.encoderAB++;
};

/**
 * \see encodersInit
 */
ISR(INT5_vect){
    if(PINA & _BV(PA0))motors.encoderCD++;
    else motors.encoderCD--;
}

/**
 * \brief Refreshes the data about current speed and driven distance.
 */
ISR(TIMER4_COMPB_vect){
    bufferA.add(motors.encoderAB);
    bufferB.add(motors.encoderCD);
    if(bufferA.filled() && bufferB.filled()){
        volatile float tmp =-(motors.encoderAB-bufferA.pop());
        if(tmp>3)tmp-=3;
        if(tmp<-3)tmp+=3;
        motors.speedAB = (tmp*ANGLE_PER_TICK)/(MOTOR_SAMPLE_TIME*10);     //see the comments of the macro definition
        motors.speedCD = ((motors.encoderCD-bufferB.pop())*ANGLE_PER_TICK)/(MOTOR_SAMPLE_TIME*10);
        motors.averageSpeed = (motors.speedAB + motors.speedCD)/2.0;
        motors.oldSpeed = motors.averageSpeed;
        motors.totalDist += (motors.averageSpeed*WHEEL_RADIUS)*MOTOR_SAMPLE_TIME;
    }
    TCNT4 = 0;                  //timer has to be reset
}

