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
#include "mpu6050_IIC.h"
#include "pid.h"
#include "motorControl.h"
#include "kalman.h"

volatile char direction[3];
volatile uint8_t tracking=0;
volatile uint8_t directionReceived=0;

int main(void){
    initUART();                                     //set baudrate (BAUD) in uart.h if change is needed
    initIIC();                                      //initializes TWI interface
    MotorDrive motors(&PORTB,&PORTB,0,1,2,3);       //motors controlled by PB1,PB2,PB3,PB4
    motors.initMotors();
    PID MPU(20,0,0);                                //initiates PID for motors
    sei();              //need to use interrupts to read data








    return 0;
}


