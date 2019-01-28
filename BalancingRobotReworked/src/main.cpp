/**
 * \file main.cpp
 * \author Daniel Adamkovic
 * \brief Main code controlling the robots operation.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "mpu6050_IIC.h"
#include "pid.h"
#include "motorControl.h"
#include "timeTracking.h"

#define ANGLE_OFFSET -0.8          //less here means more forward
#define DEBUG_OUTPUT 1             //set to 0 to disable debugging info
#define SERVO_OFFSET -8
#define UART_BAUD_RATE 9600

MotorControl motors(&DDRC,&DDRC,&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4
float giveGyroAngle(float XAngle, float YAngle, float XChange, float YChange, float ZChange, float dt, char c);
float constrain(float x, float minValue, float maxValue);


int main(void){
    float MPUData[7];                           //will contain data from MPU
    float dt,longDt;
    uint16_t counter = 0;
    float accXAngle, gyroXAngle, compXAngle;
    float accYAngle, gyroYAngle, compYAngle;
    float gyroXChange, gyroYChange, gyroZChange;
    float gyroXDt, gyroYDt;
    float desiredAngle = 0;
    float motorPower = 0;
    float adcVal = 0;
    motors.initMotors();                                    //initiates motors
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
    sei();
    encodersInit();
    if(initIIC()==1){
        BUZZER_ON;
    }
    initServo();


/*
 *       motorSpeed           compXAngle
 *          |                     |
 *          |                     |
 *
 * ------------>speedAnglePID----->anglePwmPID
 */
    //PID anglePID(13.14,0.14,0.0354);                //initiates PID for anlge 13.14,0.14,0.0354
    //PID servoPID(2,0,0);                        //pid for servos 0,1.5,0.1

                                                       //18.5,0,0.2


                                                    //need to use interrupts to read data from UART

    clockInit();                                    //initializes time tracking
    clockStart();
    _delay_ms(1000);
    IICReadMPU(MPUData,0);

     // Everything below is there just to set initial conditions for angle measurements

    dt = clockTime();   //gets initial dt

    setServoAngle(SERVO_OFFSET);

    compXAngle = MPUData[0];
    compYAngle = MPUData[1];                        //sets initial condition for complementary filter
    accXAngle = MPUData[0];
    accYAngle = MPUData[1];
    gyroXAngle = MPUData[0];
    gyroYAngle = MPUData[1];
    _delay_ms(1000);
    compYAngle = 0.93 * (compYAngle + gyroYAngle) + 0.07 * accYAngle;   //serves for sideways orientation
    compXAngle = 0.9 * (compXAngle + gyroXAngle) + 0.1 * accXAngle;   //serves for foward-backward orientation
    clockReset();


    PID speedAnglePID(0.45,0.05,0.02);                  //0.69,0.03,0.02
    PID anglePwmPID(15.27,0.0,0.66);                        //38,0.24

    while(1){
        IICReadMPU(MPUData,0);
        dt = clockTime();
        clockReset();

        accXAngle = MPUData[0];
        accYAngle = MPUData[1];
        gyroXChange = MPUData[2];
        gyroYChange = MPUData[3];
        gyroZChange = MPUData[4];

        gyroXDt = giveGyroAngle(compXAngle, compYAngle, gyroXChange, gyroYChange, gyroZChange, dt, 'X');
        gyroYDt = giveGyroAngle(compXAngle, compYAngle, gyroXChange, gyroYChange, gyroZChange, dt, 'Y');
        gyroXAngle += gyroXDt;
        gyroYAngle += gyroYDt;


        compXAngle = (0.98 * (compXAngle + gyroXDt) + 0.02 * accXAngle);   //serves for foward-backward orientation
        compYAngle = (0.95 * (compYAngle + gyroYDt) + 0.05* accYAngle);      //serves for sideways orientation

        //adcVal = anglePwmPID.tunePID('D',00.3);

        if(counter%6 == 0){
            float currSpeed = motors.averageSpeed;
            desiredAngle = speedAnglePID.giveOutput(currSpeed,0,longDt,50);
            //desiredAngle = constrain(desiredAngle+ANGLE_OFFSET,-15,15);
            desiredAngle=desiredAngle+ANGLE_OFFSET;
            longDt = 0;
        }
        motorPower= anglePwmPID.giveOutput(compXAngle*RAD_TO_DEG,desiredAngle,dt,0);        //calling PID to give us value for motors
        motorPower = constrain(motorPower,-100,100);  //constraining PID output

        //servoAngle = servoPID.giveOutput(compYAngle,0,dt,18000);
        //servoAngle = constrain(servoAngle,-20,20);
        //setServoAngle(servoAngle*RAD_TO_DEG+SERVO_OFFSET);

        //setServoAngle(SERVO_OFFSET);
        motors.SetSpeedBoth((int8_t)motorPower);
        //if(longDt>dt*5)uart_putf(longDt*10);

        _delay_ms(10);

        if((counter == 201) & (DEBUG_OUTPUT == 1)){

            uart_puts("Constant is: ");
            uart_putf(desiredAngle);
            uart_putc('\n');
            uart_puts("Motor speed is: ");
            uart_putf(motors.averageSpeed);
            //uart_puti(((int16_t)(gyroXAngle*RAD_TO_DEG)));
             uart_putc('\n');
             uart_putc('\n');
             uart_putc('\n');
             //if((compXAngle*RAD_TO_DEG)>10)BUZZER_ON;
            counter=0;
    }

    else counter++;
    longDt+=dt;

    }


    return 0;
}

float giveGyroAngle(float XAngle, float YAngle, float XChange, float YChange, float ZChange, float dt, char c){
    if(c=='X')return -(XChange + YChange*((sin(XAngle)*sin(YAngle))/cos(YAngle))+ ZChange*((cos(XAngle)*sin(YAngle))/(cos(YAngle))))*dt;
    else if(c=='Y')return -((YChange*cos(XAngle)) - ZChange*sin(XAngle))*dt;
    return 0;
}

float constrain(float x, float minValue, float maxValue){
    if(x < minValue)return minValue;
    else if(x > maxValue)return maxValue;
    else return x;
}


