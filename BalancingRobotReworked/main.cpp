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
#include "timeTracking.h"

#define ANGLE_OFFSET 2.2
#define SPEED_OFFSET 25

int8_t constrain(int8_t,int8_t,int8_t);

volatile char direction[3];
volatile uint8_t tracking=0;
volatile uint8_t directionReceived=0;


int main(void){

    double accYAngle, gyroYAngle, compYAngle, dt;
    int8_t motorSpeed;


    initUART();                                     //set baudrate (BAUD) in uart.h if change is needed
    initIIC();                                      //initializes TWI interface
    MotorDrive motors(&PORTB,&PORTB,0,1,2,3);       //motors controlled by PB1,PB2,PB3,PB4
    motors.initMotors();                            //initiates motors
    PID MPU(20,0,0);                                //initiates PID for motors
    sei();                                          //need to use interrupts to read data from UART
    uint16_t MPUData[7];                            //will contain data from MPU
    clockInit();                                    //initializes time tracking
    clockStart();
    IICreadMPU(MPUData,0);


     // Everything below is there just to set initial conditions for angle measurements

    dt = clockTime();                               //gets initial dt
    compYAngle = MPUData[1];                        //sets initial condition for complementary filter
    accYAngle = MPUData[1];
    gyroYAngle = MPUData[3]*dt;
    compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;
    clockReset();

    while(1){
        IICreadMPU(MPUData,0);

        compYAngle = MPUData[1];
        accYAngle = MPUData[1];
        dt = clockTime();
        gyroYAngle += MPUData[3]*dt;
        compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;


        motorSpeed = MPU.giveOutput(compYAngle + ANGLE_OFFSET,0,dt);        //calling PID to give us value for motors
        motorSpeed = constrain(motorSpeed,-100,100);                        //constraining PID output
        motors.SetSpeedBoth(motorSpeed + SPEED_OFFSET);                     //setting new speed, offset necessary (motor needs some voltage to move)


        clockReset();

    }









    return 0;
}

/*
 * INPUT : x = value to constrain, minValue - maxValue = constrain borders
 * OUTPUT :
 */

int8_t constrain(int8_t x, int8_t minValue, int8_t maxValue){
    if(x < minValue)return minValue;
    else if(x > maxValue)return maxValue;
    else return x;
}

