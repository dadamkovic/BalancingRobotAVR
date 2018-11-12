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
static uint16_t counter = 0;
float MPUData[7];                            //will contain data from MPU

void encodersInit();                        //didn't include this in any other header file
void initServo();
uint8_t setServoAngle(float);




#endif /* MAIN_H_ */
