/*
 * main.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */



#include "main.h"

#define DEBUG_OUTPUT 1



MotorDrive motors(&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4
int main(void){
    float accYAngle, gyroYAngle, compYAngle, dt;
    int8_t motorSpeed;


    DDRH |= _BV(PH4)|_BV(PH5);
    PORTH |= _BV(PH4) | _BV(PH5);
    initControllerUART();                           //set baudrate (BAUD) in uart.h if change is needed
    initInterfaceUART();
    encodersInit();
    initIIC();                                      //initializes TWI interface

    motors.initMotors();                            //initiates motors
    PID MPU(20,0,0);                                //initiates PID for motors
    sei();                                          //need to use interrupts to read data from UART

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
        dt = clockTime();
        accYAngle = MPUData[1];

        clockReset();
        gyroYAngle -= (MPUData[3] * dt);
        compYAngle = 0.90 * (compYAngle + MPUData[3]*dt) + 0.1 * accYAngle;

        motorSpeed = MPU.giveOutput(compYAngle + ANGLE_OFFSET,0,dt);        //calling PID to give us value for motors
        motorSpeed = constrain(motorSpeed,-100,100);                        //constraining PID output
        motors.SetSpeedBoth(motorSpeed + SPEED_OFFSET);                     //setting new speed, offset necessary (motor needs some voltage to move)

  if((counter == 200) & (DEBUG_OUTPUT == 1)){
        interfaceSendString("Dt was: ");
        interfaceSendFloat(dt);
        interfaceSendString("\naccAngle is: ");
        interfaceSendFloat(MPUData[1]);
        interfaceSendString("\nGyroAngle is:");
        interfaceSendFloat(gyroYAngle);
        interfaceSendChar('\n');
        interfaceSendString("\nCOMPAngle is:");
        interfaceSendFloat(compYAngle);
        interfaceSendChar('\n');
        counter=0;
    }
    else counter++;
    //_delay_ms(10);

    }


    return 0;
}



