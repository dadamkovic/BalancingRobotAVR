/*
 * main.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */



#include "main.h"
#include <stdio.h>



MotorDrive motors(&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4
int main(void){

    double accYAngle, gyroYAngle, compYAngle, dt;
    int8_t motorSpeed;

    initControllerUART();                           //set baudrate (BAUD) in uart.h if change is needed
    initInterfaceUART();
    encodersInit();
    //initIIC();                                      //initializes TWI interface

    motors.initMotors();                            //initiates motors
    PID MPU(20,0,0);                                //initiates PID for motors
    sei();                                          //need to use interrupts to read data from UART
    uint16_t MPUData[7];                            //will contain data from MPU
    clockInit();                                    //initializes time tracking
    clockStart();
    //IICreadMPU(MPUData,0);


     // Everything below is there just to set initial conditions for angle measurements

    dt = clockTime();                               //gets initial dt
/*    compYAngle = MPUData[1];                        //sets initial condition for complementary filter
    accYAngle = MPUData[1];
    gyroYAngle = MPUData[3]*dt;
    compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;*/
    clockReset();

    while(1){
        /*IICreadMPU(MPUData,0);

        compYAngle = MPUData[1];
        accYAngle = MPUData[1];*/
        dt = clockTime();
        /*gyroYAngle += MPUData[3]*dt;
        compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;


        motorSpeed = MPU.giveOutput(compYAngle + ANGLE_OFFSET,0,dt);        //calling PID to give us value for motors
        motorSpeed = constrain(motorSpeed,-100,100);                        //constraining PID output
        motors.SetSpeedBoth(motorSpeed + SPEED_OFFSET);*/                     //setting new speed, offset necessary (motor needs some voltage to move)



        //sprintf(buffer,"%f",speedAB);


        /*interfaceSendString("Motor speed was: ");
        interfaceSendChar(speedAB);*/
        interfaceSendString("Tick is: ");
        interfaceSendChar(48+encoderAB);
        interfaceSendChar('\r');
        clockReset();
        _delay_ms(500);

    }

    return 0;
}




