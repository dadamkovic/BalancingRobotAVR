/*
 * main.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */



#include "main.h"

#define DEBUG_OUTPUT 1

//testing branching

MotorDrive motors(&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4
int main(void){
    DDRG |= _BV(PG2);
    PORTG |= _BV(PG2);
    float accYAngle, gyroYAngle, compYAngle, dt;
    int8_t motorSpeed;
    DDRC |= (_BV(PC0)|_BV(PC2)|_BV(PC4)|_BV(PC6));

    DDRH |= _BV(PH4)|_BV(PH5);
    PORTH |= _BV(PH4) | _BV(PH5);
    initControllerUART();                           //set baudrate (BAUD) in uart.h if change is needed
    initInterfaceUART();
    encodersInit();
    initIIC();                                      //initializes TWI interface
    initServo();
    motors.initMotors();                            //initiates motors
    PID anglePID(20,0,0);                                //initiates PID for motors
    PID servoPID(0,3,0);
    sei();                                          //need to use interrupts to read data from UART

    clockInit();                                    //initializes time tracking
    clockStart();
    IICreadMPU(MPUData,0);

     // Everything below is there just to set initial conditions for angle measurements

    dt = clockTime();                               //gets initial dt
    compYAngle = MPUData[1];                        //sets initial condition for complementary filter
    accYAngle = MPUData[1];
    if(accYAngle>90)accYAngle = 180 - accYAngle;
    if(accYAngle<-90)accYAngle = -180 - accYAngle;
    gyroYAngle = MPUData[3]*dt;
    compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;
    clockReset();
    while(1){
        IICreadMPU(MPUData,0);
        dt = clockTime();
        accYAngle = MPUData[1];
        if(accYAngle>90)accYAngle = 180 - accYAngle;
        if(accYAngle<-90)accYAngle = -180 - accYAngle;
        clockReset();
        gyroYAngle -= (MPUData[3] * dt);
        compYAngle = 0.90 * (compYAngle + MPUData[3]*dt) + 0.1 * accYAngle;
        float servoAngle = servoPID.giveOutput(compYAngle + ANGLE_OFFSET,0,dt,15000);
        motorSpeed = anglePID.giveOutput(compYAngle + ANGLE_OFFSET,0,dt,0);        //calling PID to give us value for motors
        motorSpeed = constrain(motorSpeed,-100,100);                        //constraining PID output
        //motors.SetSpeedBoth((int8_t)motorSpeed + SPEED_OFFSET);                     //setting new speed, offset necessary (motor needs some voltage to move)
  if((counter == 2000) & (DEBUG_OUTPUT == 1)){
        interfaceSendString("Speed was: ");
        interfaceSendFloat((int8_t)motorSpeed);
        interfaceSendString("\nAccAngle is: ");
        interfaceSendFloat(accYAngle);
        interfaceSendString("\nGyroAngle is:");
        interfaceSendFloat(gyroYAngle);
        interfaceSendChar('\n');
        interfaceSendString("\nCOMPAngle is:");
        interfaceSendFloat(compYAngle);
        interfaceSendChar('\n');
        counter=0;

    }

    else counter++;
    servoAngle = constrain(servoAngle,-70,70);
    setServoAngle(-servoAngle);
    //_delay_ms(10);

    }


    return 0;
}



