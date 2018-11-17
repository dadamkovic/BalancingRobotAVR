/*
 * main.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */





#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "mpu6050_IIC.h"
#include "pid.h"
#include "motorControl.h"
#include "timeTracking.h"

#define ANGLE_OFFSET 1.5          //less here means more forward
#define SPEED_OFFSET 8
#define DEBUG_OUTPUT 1          //set to 0 to disable debugging info
#define SERVO_OFFSET -8



volatile char controllerData[3] = {'\0','\0','\0'};   //initialized to all '\0'            //
volatile uint8_t receivingCommandFlag=0;
static uint16_t counter = 0;
float MPUData[7];                            //will contain data from MPU

void encodersInit();                        //didn't include this in any other header file
void initServo();
uint8_t setServoAngle(float);


//testing branching

MotorDrive motors(&DDRC,&DDRC,&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4



/*
 * DEFINES AND DECLARATIONS END HERE
 * ************************************************************************************
 */

int main(void){
    float accXAngle, gyroXAngle, compXAngle;
    float accYAngle, gyroYAngle, compYAngle, dt;
    float gyroXChange, gyroYChange, gyroZChange;
    float gyroXDt, gyroYDt;
    float servoAngle=0;
    float motorSpeed;
    DDRH |= _BV(PH4)|_BV(PH5);
    PORTH |= _BV(PH4) | _BV(PH5);
    //initControllerUART();                           //set baudrate (BAUD) in uart.h if change is needed
    //initBLEModul();
    motors.initMotors();                            //initiates motors
    initInterfaceUART();
    encodersInit();
    initIIC();                                      //initializes TWI interface
    initServo();
    PID anglePID(2.2,2,2);                           //initiates PID for motors
    PID servoPID(0,1.5,0.2);                        //pid for servos 0,1.5,0.1
    sei();                                          //need to use interrupts to read data from UART

    clockInit();                                    //initializes time tracking
    clockStart();
    _delay_ms(500);
    IICReadMPU(MPUData,0);

     // Everything below is there just to set initial conditions for angle measurements

    dt = clockTime();                               //gets initial dt

    compXAngle = MPUData[0];
    compYAngle = MPUData[1];                        //sets initial condition for complementary filter
    accXAngle = MPUData[0];
    accYAngle = MPUData[1];
    gyroXAngle = MPUData[0];
    gyroYAngle = MPUData[1];

    compYAngle = 0.9 * (compYAngle + gyroYAngle) + 0.1 * accYAngle;   //serves for sideways orientation
    compXAngle = 0.98 * (compXAngle + gyroXAngle) + 0.02 * accXAngle;   //serves for foward-backward orientation
    clockReset();
    setServoAngle(-accYAngle);
    while(1){
        IICReadMPU(MPUData,0);

        dt = clockTime();

        accXAngle = MPUData[0];
        accYAngle = MPUData[1];
        gyroXChange = MPUData[2];
        gyroYChange = MPUData[3];
        gyroZChange = MPUData[4];

        gyroXDt = (gyroXChange + gyroYChange*((sin(compXAngle)*sin(compYAngle))/cos(compYAngle)) + \
                     + gyroZChange*((cos(compXAngle)*sin(compYAngle))/(cos(compYAngle))))*dt;
        gyroYDt = ((gyroYChange*cos(compXAngle)) - gyroZChange*sin(compXAngle))*dt;
        gyroXAngle += gyroXDt;
        gyroYAngle += gyroYDt;

        clockReset();

        compXAngle = 0.93 * (compXAngle + gyroXDt) + 0.07 * accXAngle;   //serves for foward-backward orientation
        compYAngle = 0.95 * (compYAngle + gyroYDt) + 0.05* accYAngle;   //serves for sideways orientation



        motorSpeed = anglePID.giveOutput(compXAngle + ANGLE_OFFSET,0,dt,0);        //calling PID to give us value for motors
        servoAngle = servoPID.giveOutput(compYAngle,0,dt,18000);


        motorSpeed = constrain(motorSpeed,-100,100);   //constraining PID output
        servoAngle = constrain(servoAngle,-40,40);
        setServoAngle(-servoAngle+SERVO_OFFSET);

        motors.SetSpeedBoth(((int8_t)motorSpeed*-1)+SPEED_OFFSET);                     //setting new speed, offset necessary (motor needs some voltage to move);

        if((counter == 2001) & (DEBUG_OUTPUT == 1)){
            interfaceSendString("Mspeeed was: ");
            //interfaceSendFloat(motors.speedAB);
            interfaceSendString("\nAccAngle is: ");
            interfaceSendFloat(accXAngle);
            interfaceSendString("\nGyroAngle is:");
            interfaceSendFloat(gyroXAngle);
            interfaceSendChar('\n');
            interfaceSendString("\nCOMPAngle is:");
            interfaceSendFloat(compXAngle);
            interfaceSendChar('\n');
            counter=0;
    }

    else counter++;

    }


    return 0;
}



