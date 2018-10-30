/*
 * main.h
 *
 *  Created on: Oct 20, 2018
 *      Author: daniel
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "uart.h"
#include "mpu6050_IIC.h"
#include "pid.h"
#include "motorControl.h"
#include "timeTracking.h"

#define ANGLE_OFFSET 2.2
#define SPEED_OFFSET 25


volatile char controllerData[3] = {'\0','\0','\0'};   //initialized to all '\0'            //
volatile uint8_t receiveCommandFlag=0;
volatile int16_t encoderAB, encoderCD = 0;
volatile float speedAB,speedCD = 0;
static uint8_t counter = 0;
float MPUData[7];                            //will contain data from MPU

void encodersInit();                        //didn't include this in any other header file


/*
 * INPUT : x = value to constrain, minValue - maxValue = constrain borders
 * OUTPUT :
 */

int8_t constrain(int8_t x, int8_t minValue, int8_t maxValue){
    if(x < minValue)return minValue;
    else if(x > maxValue)return maxValue;
    else return x;
}


#endif /* MAIN_H_ */
