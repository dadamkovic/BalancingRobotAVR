/*
 * servo.cpp
 *
 *  Created on: Nov 2, 2018
 *      Author: daniel
 */

#include <avr/io.h>
#include "inttypes.h"

void initServo(){
    TCCR3A &= ~(_BV(WGM30));
    TCCR3A |= (_BV(WGM31));
    TCCR3B |= (_BV(WGM32)|_BV(WGM33));                 //Fast PWM mode TOP at ICR3
    DDRE |= _BV(PE3);

    OCR3A = 3000;                   //servo midpoint
    ICR3 = 39999;                   //frequency 50Hz
    TCCR3A |= _BV(COM3A1);               //OCR3 clear on compare, set on BOT

    TCCR3B |= _BV(CS31);
    TCCR3B &= ~(_BV(CS30)|_BV(CS32));      //prescaler 8

}


uint8_t setServoAngle(float target){
    //if((target<-99) | (target>99))return 1;
    float timingRegister = (target/90)*1000;
    OCR3A = 3000 + (int16_t)timingRegister;
    return 0;
}

