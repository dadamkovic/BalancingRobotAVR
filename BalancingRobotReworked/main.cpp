/*
 * main.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */




#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "uart.h"

volatile char direction[3];
volatile uint8_t tracking=0;
volatile uint8_t directionReceived=0;

int main(void){
    initUART();     //set baudrate (BAUD) in uart.h if change is needed
    sei();          //need to use interrupts to read data





    return 0;
}


