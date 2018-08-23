/*
 * main.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: daniel
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "init_functions.h"

int main(void){
    //sei();

    initUART();
    _delay_ms(500);
    initBLUETOOTH();
    initPINS();
    while(1){
        sendCommand();
        _delay_ms(100);
    }
    return 0;
}

