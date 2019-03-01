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
#include "robotControl.h"
#include "utility.h"

#define ANGLE_OFFSET 0
#define DEBUG_OUTPUT 0            //set to 0 to disable debugging info
#define DATA_LOGGING 0
#define SERVO_OFFSET -8
#define UART_BAUD_RATE 57600
#define MSB 0xff00
#define LSB 0x00ff


MotorControl motors(&DDRC,&DDRC,&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4
float constrain(float x, float minValue, float maxValue);
float map(float num2map, float botInit, float topInit, float mapLow, float mapHigh);
uint8_t resolveCommand(uint8_t *command, PID *pid, MotorControl *motorsC, MPU *mpu);

int main(void){
    ROBOT_LED_ON;
    float dt,longDt = 0;
    uint16_t counter = 0;
    float motorPower = 0;
    float desiredAngle = 0;

    uint8_t command = 0x00;
    int8_t toggle_transmission = -1;

    motors.initMotors();                                    //initiates motors
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
    uart3_init(UART_BAUD_SELECT(57600, F_CPU));
    sei();
    encodersInit();
    BUZZER_INIT;
    //BUZZER_ON;
    //_delay_ms(1000);
    //BUZZER_OFF;
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
    MPU mpu6050;

    dt = clockTime();   //gets initial dt

    setServoAngle(mpu6050.compYAngle+SERVO_OFFSET);
    clockReset();
    _delay_ms(10);


    //PID speedAnglePID(0.45,0.05,0.02);                  //0.69,0.03,0.02
    PID speedAnglePID(0.45,0.05,0.02);
    //PID anglePwmPID(15.27,0.0,0.66);                        //38,0.24
    PID anglePwmPID(15.27,0.0,0.66);
    while(1){

        dt = clockTime();
        clockReset();
        mpu6050.updateValues(dt);

        //adcVal = anglePwmPID.tunePID('D',00.3);

        if(counter%6 == 0){
            float currSpeed = motors.averageSpeed;
            float desiredSpeed = motors.desiredSpeed;
            desiredAngle = speedAnglePID.giveOutput(currSpeed,desiredSpeed,longDt,50);
            desiredAngle = constrain(desiredAngle+ANGLE_OFFSET,-10,10);
            desiredAngle=desiredAngle+ANGLE_OFFSET;
            longDt = 0;
        }
        motorPower= anglePwmPID.giveOutput(mpu6050.compXAngle*RAD_TO_DEG,desiredAngle,dt,0);        //calling PID to give us value for motors
        motorPower = constrain(motorPower,-100,100);  //constraining PID output

        //servoAngle = servoPID.giveOutput(compYAngle,0,dt,18000);
        //servoAngle = constrain(servoAngle,-20,20);
        //setServoAngle(servoAngle*RAD_TO_DEG+SERVO_OFFSET);

        //setServoAngle(SERVO_OFFSET);
        motors.setSpeedIndividually((int8_t)motorPower);

        //motors.motorASpeedOffset = (0.999999*motors.motorASpeedOffset + 0.000001*0);
        //motors.motorBSpeedOffset = -(motors.motorASpeedOffset);
        uart_putf(motors.motorASpeedOffset);
        uart_putc('\n');
        while(clockTime()<0.01){
            if(uart_available()){
                if(uart_getc()=='X'){
                toggle_transmission *= -1;
                }
            }
            if(uart3_available()){
                command = uart3_getc();
                //uart_puti(command);
                //uart_putc('\n');
                resolveCommand(&command, &anglePwmPID, &motors, &mpu6050);
            }
            motors.updateBatteryLvl();
        }

        if((counter == 201) & (DEBUG_OUTPUT == 1)){

            uart_puts("Motors: ");
            uart_putf(motors.getBatteryLvl());
            uart_putc('\n');
            uart_puts("Time delta is: ");
            uart_putf(dt);
            uart_putc('\n');
             uart_putc('\n');
             uart_putc('\n');
             uart_putc('\n');
            counter=0;
    }
    if((counter % 5 == 0) && DATA_LOGGING && (toggle_transmission>0)){
        uart_putf(mpu6050.compXAngle);
        uart_putc(',');
        uart_putf(mpu6050.gyroXAngle);
        uart_putc(',');
        uart_putf(mpu6050.compYAngle);
        uart_putc(',');
        uart_putf(mpu6050.gyroYAngle);
        uart_putc(',');
        uart_putf(motors.averageSpeed);
        uart_putc(',');
        uart_putf(motors.getBatteryLvl());
        uart_putc('\n');
    }
    counter++;
    if(counter>1000)counter=0;
    longDt+=dt;

    }


    return 0;
}



uint8_t resolveCommand(uint8_t *command, PID *pid, MotorControl *motorsC, MPU *mpu){
    //positional starts with 0b1xxx, no others do
    if(*command & _BV(7)){
        uint8_t steering = ((*command & 0b01110000)>>4);
        uint8_t throttle = (*command & 0b00001111);
        float newSpeed = map(throttle,0,15, -10, 16)-5.6;
        float newSteering = map(steering,0,7,-20,20);
        newSteering = constrain(newSteering, -14,14);
        motorsC->desiredSpeed = constrain((0.98*motorsC->desiredSpeed + 0.02*newSpeed),-10,10);
        motorsC->motorASpeedOffset = newSteering;
        motorsC->motorBSpeedOffset = -(motorsC->motorASpeedOffset);
        *command = 0x00;
        return 0;
    }
    uint8_t tmp;
    switch(*command){
    case REQ_BATTERY_LVL:
        uart3_putc(motorsC->getBatteryLvl());
        *command = 0x00;
        break;
    case REQ_TILT_ANGLE:
        int16_t angle;
        angle = (int16_t)(mpu->compXAngle*100);
        tmp = (angle>>8);
        uart3_putc(tmp);
        tmp = (angle&LSB);
        uart3_putc(tmp);
        *command = 0x00;
        break;
    case REQ_SPEED:
        int16_t speed;
        speed = (int16_t) (motorsC->averageSpeed*100);
        tmp = speed>>8;
        uart3_putc(tmp);
        tmp = (speed&LSB);
        uart3_putc(tmp);
        *command = 0x00;
        break;
    }
        return 0;
    }


