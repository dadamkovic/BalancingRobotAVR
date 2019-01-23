/**
 * \author Daniel Adamkovic
 * \date 22/1/2019
 * \file servo.cpp
 * \brief Functions that handle control of the robot's servo
 * \bugs Servo timings were set to work, but don't seem to follow the 20ms/1.5ms range.
 */

#include <avr/io.h>
#include "inttypes.h"

/**
 * \brief Sets up all timers and pins that control the servo
 * \return void
 */
void initServo(){
    TCCR3A &= ~(_BV(WGM30));
    TCCR3A |= (_BV(WGM31));
    TCCR3B |= (_BV(WGM32)|_BV(WGM33));                 //Fast PWM mode TOP at ICR3
    DDRE |= _BV(PE3);

    OCR3A = 3000;                                       //servo midpoint
    ICR3 = 39999;                                       //frequency 50Hz
    TCCR3A |= _BV(COM3A1);                              //OCR3 clear on compare, set on BOT

    TCCR3B |= _BV(CS31);
    TCCR3B &= ~(_BV(CS30)|_BV(CS32));                   //prescaler 8

}

/**
 * \brief Used for setting the current requested servo angle
 * \param[in] target Angle in degrees, where 0 means midpoint.
 * \return Returns 0.
 * \note Maybe chage to return void, 0 serves no purpoupse at all.
 */
uint8_t setServoAngle(float target){
    //if((target<-99) | (target>99))return 1;
    float timingRegister = (target/90)*2000;
    OCR3A = 3000 + (int16_t)timingRegister;
    return 0;
}

