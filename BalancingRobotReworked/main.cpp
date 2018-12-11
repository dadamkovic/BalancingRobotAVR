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

#define ANGLE_OFFSET -0.28          //less here means more forward -0.38
#define SPEED_OFFSET 2
#define DEBUG_OUTPUT 1           //set to 0 to disable debugging info
#define SERVO_OFFSET -8

void encodersInit();                        //didn't include this in any other header file
void initServo();
uint8_t setServoAngle(float);

volatile char controllerData[3] = {'\0','\0','\0'};   //initialized to all '\0'            //
volatile uint8_t receivingCommandFlag=0;
static uint16_t counter = 0;
float MPUData[7];                            //will contain data from MPU
float calVal[5];
float dt;



//testing branching

MotorDrive motors(&DDRC,&DDRC,&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4



/*
 * DEFINES AND DECLARATIONS END HERE
 * ************************************************************************************
 */

int main(void){
    float accXAngle, gyroXAngle, compXAngle;
    float accYAngle, gyroYAngle, compYAngle;
    float gyroXChange, gyroYChange, gyroZChange;
    float gyroXDt, gyroYDt;
    float servoAngle=0;
    float desiredAngle,motorPower;
    float ADCval, filteredSpeed = 0 ;

    ADMUX |= _BV(REFS0) | _BV(REFS1);   //2.2 V reference
    ADCSRA |= _BV(ADEN);                //ADC initialized

    //DDRH |= _BV(PH4)|_BV(PH5);
    //PORTH |= _BV(PH4) | _BV(PH5);

    motors.initMotors();                            //initiates motors
    initInterfaceUART();
    encodersInit();
    initIIC();                                      //initializes TWI interface
    initServo();


/*
 *       motorSpeed           compXAngle
 *          |                     |
 *          |                     |
 *
 * ------------>speedAnglePID----->anglePwmPID
 */
    //PID anglePID(13.14,0.14,0.0354);                //initiates PID for anlge 13.14,0.14,0.0354
    PID servoPID(0,1.5,0.2);                        //pid for servos 0,1.5,0.1

    PID speedAnglePID(0.5115,0,0);
    PID anglePwmPID(18.5,0,0.2);        //38,0.24
    PID accelPwmPID(0,0,0);
    //Calibration vals
    /*float bufferAll[5];
    DDRL |= _BV(PL0);
    PORTL |= _BV(PL0);
    for(uint16_t i=0;i<1000;i++){
            IICReadMPU(bufferAll,0);
            for(uint8_t j=0;j<5;j++)calVal[j]+=bufferAll[j]/1000.0;
            _delay_ms(2);
    }
    PORTL &= ~_BV(PL0);*/
    /*
    calVal[0] = -0.6893;
    calVal[1] = -1.1260;
    calVal[2] = -0.726;
    calVal[3] = -0.0486;
    calVal[4] = -0.4103;
    */
    //-----


    sei();                                          //need to use interrupts to read data from UART

    clockInit();                                    //initializes time tracking
    clockStart();
    IICReadMPU(MPUData,0);

     // Everything below is there just to set initial conditions for angle measurements

    dt = clockTime();                               //gets initial dt

    compXAngle = MPUData[0];
    compYAngle = MPUData[1];                        //sets initial condition for complementary filter
    accXAngle = MPUData[0];
    accYAngle = MPUData[1];
    gyroXAngle = MPUData[0];
    gyroYAngle = MPUData[1];

    compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;   //serves for sideways orientation
    compXAngle = 0.98 * (compXAngle + gyroXAngle) + 0.02 * accXAngle;   //serves for foward-backward orientation
    clockReset();
    setServoAngle(SERVO_OFFSET);
    while(1){

        IICReadMPU(MPUData,0);

        dt = clockTime();
        clockReset();

        accXAngle = MPUData[0];
        accYAngle = MPUData[1] - calVal[1]*DEG_TO_RAD;
        gyroXChange = MPUData[2];
        gyroYChange = MPUData[3];
        gyroZChange = MPUData[4];

        gyroXDt = (gyroXChange + gyroYChange*((sin(compXAngle)*sin(compYAngle))/cos(compYAngle))+ gyroZChange*((cos(compXAngle)*sin(compYAngle))/(cos(compYAngle))))*dt;
        gyroYDt = ((gyroYChange*cos(compXAngle)) - gyroZChange*sin(compXAngle))*dt;
        gyroXAngle -= gyroXDt;
        gyroYAngle -= gyroYDt;


        compXAngle = (0.998 * (compXAngle - gyroXDt) + 0.002 * accXAngle);   //serves for foward-backward orientation
        compYAngle = (0.95 * (compYAngle - gyroYDt) + 0.05* accYAngle);   //serves for sideways orientation

        //ADCval = speedAnglePID.tunePID('P');

        if((counter % 6 )== 0){
            filteredSpeed = (0.6 * (filteredSpeed) + 0.4* motors.averageSpeed);
            desiredAngle = speedAnglePID.giveOutput(filteredSpeed,0,dt,0);
        }
        desiredAngle = constrain(desiredAngle,-5,5);
        motorPower= anglePwmPID.giveOutput(compXAngle*RAD_TO_DEG+ANGLE_OFFSET,desiredAngle,dt,0);        //calling PID to give us value for motors
        //servoAngle = servoPID.giveOutput(compYAngle,0,dt,18000);
        motorPower = constrain(motorPower,-100,100);  //constraining PID output
        //servoAngle = constrain(servoAngle,-40,40);
        //setServoAngle(-servoAngle+SERVO_OFFSET);
        setServoAngle(SERVO_OFFSET);

        if(motorPower>0)motors.SetSpeedBoth((int8_t)motorPower+SPEED_OFFSET);
        else if(motorPower<0) motors.SetSpeedBoth((int8_t)motorPower-SPEED_OFFSET);                  //setting new speed, offset necessary (motor needs some voltage to move);


        _delay_ms(10);

        if((counter == 201) & (DEBUG_OUTPUT == 1)){
            /*interfaceSendString("GyroXAngle was: ");
            interfaceSendFloat(gyroXAngle);
            interfaceSendString("\nAccXAngle is: ");
            interfaceSendFloat(accXAngle);
            interfaceSendString("\nGyroYAngle is:");
            interfaceSendFloat(gyroYAngle);
            interfaceSendString("\nAccYAngle is: ");
            interfaceSendFloat(accYAngle);
            interfaceSendChar('\n');*/
            interfaceSendString("\nCOMPAngle is:");
            interfaceSendFloat(compXAngle);
            //interfaceSendChar('\n');
            interfaceSendString("\nSpeed is: ");
            interfaceSendFloat(filteredSpeed);
            //interfaceSendChar('\n');
            interfaceSendString("\nP is: ");
            interfaceSendFloat(ADCval);
            interfaceSendChar('\n');
            interfaceSendChar('\n');
            /*for(uint8_t i =0; i<5;i++){
                interfaceSendFloat(calVal[i]);
                interfaceSendChar('\n');
            }*/
            counter=0;
    }

    else counter++;

    }


    return 0;
}




