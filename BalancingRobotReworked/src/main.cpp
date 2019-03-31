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
#define DEBUG_OUTPUT 0             //set to 0 to disable debugging info
#define DATA_LOGGING 0
#define SERVO_OFFSET -8
#define UART_BAUD_RATE 57600
#define LSB 0x00ff
#define MSB 0xff00

float EEMEM xCalAddr;
float EEMEM yCalAddr;
float EEMEM zCalAddr;

MotorControl motors(&DDRC,&DDRC,&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4
float constrain(float x, float minValue, float maxValue);
float map(float num2map, float botInit, float topInit, float mapLow, float mapHigh);
uint8_t resolveCommand(uint8_t *movementBuffer,uint8_t *command, uint8_t *controlMovement, PID *pid, MotorControl *motorsC, MPU *mpu);


int main(void){
    ROBOT_LED_ON;
    float dt,longDt = 0;
    uint16_t counter = 0;
    float motorPower = 0;
    float desiredAngle = 0;
    uint8_t bufferIndex = 0;
    uint8_t commandBuffer[2] = {0,0};
    uint8_t command = 0x00;
    int8_t toggle_transmission = -1;        //used to stop transmitting when PC closes port
    uint8_t controlMovement = 1;
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

    setServoAngle(mpu6050.compYAngle*RAD_TO_DEG+SERVO_OFFSET);
    clockReset();
    _delay_ms(10);


    //PID speedAnglePID(0.55,0.05,0.02);                  //0.69,0.03,0.02
    PID speedAnglePID(0.55,0.00,0.1);
    //PID anglePwmPID(15.27,0.0,0.66);                        //38,0.24
    PID anglePwmPID(15.27,0.0,0.66);
    //PIC distancePID(15,0,0);
    PID distancePID(15,0,0);
    //PID servoPID(1.3,0,0.1);


    while(1){
        dt = clockTime();
        clockReset();
        mpu6050.updateValues(dt);
        if(counter%6 == 0){
            float currSpeed = motors.averageSpeed;
            float desiredSpeed = motors.desiredSpeed;
            if((motors.desiredSpeed <0.5) && (motors.desiredSpeed > -0.5) && controlMovement){
                desiredSpeed = constrain(distancePID.giveOutput(motors.totalDist,0,longDt,0),-5,5);
            }
            desiredAngle = speedAnglePID.giveOutput(currSpeed,desiredSpeed,longDt,10);
            desiredAngle = constrain(desiredAngle,-5,5);
            longDt = 0;
        }

        motorPower= anglePwmPID.giveOutput(mpu6050.compXAngle*RAD_TO_DEG,desiredAngle,dt,0);        //calling PID to give us value for motors
        motorPower = constrain(motorPower,-90,90);  //constraining PID output

        //float servoAngle = servoPID.giveOutput(mpu6050.compYAngle*RAD_TO_DEG+SERVO_OFFSET,0,dt,0);
        //servoAngle = constrain(servoAngle,-50,50);
        //setServoAngle(servoAngle);

        //setServoAngle(SERVO_OFFSET);
        if(((mpu6050.compXAngle*RAD_TO_DEG)>20) || ((mpu6050.compXAngle*RAD_TO_DEG)<-20)){
            motors.setSpeedIndividually(0);
        }
        else{
            motors.setSpeedIndividually((int8_t)motorPower);
        }

        motors.motorSpeedOffset = (0.99999*motors.motorSpeedOffset + 0.00001*0);
        motors.desiredSpeed = (0.999999*motors.desiredSpeed + 0.000001*0);

        while(clockTime()<0.01){
            if(uart_available()){
                if(uart_getc()=='X'){
                    toggle_transmission *= -1;
                }
            }
            if(uart3_available()){
                command = uart3_getc();
                if(bufferIndex){
                    commandBuffer[bufferIndex-1] = command;
                    if(bufferIndex == 2){
                        resolveCommand(commandBuffer,&command, &controlMovement, &speedAnglePID, &motors, &mpu6050);
                        bufferIndex = 0;
                    }
                    else{
                        bufferIndex++;
                    }
                }
                if(command == CONTROL_INFO){
                    bufferIndex = 1;
                }
                else{
                    resolveCommand(commandBuffer,&command, &controlMovement, &speedAnglePID, &motors, &mpu6050);
                }
            }

        }


        if((counter == 201) && (DEBUG_OUTPUT == 1)){
            uart_putf(mpu6050.xCal);
            uart_putc('\n');
            uart_putf(dt);
            uart_putc('\n');
            //uart_putc('\n');
            counter=0;
        }
        if(((counter % 5) == 0) && DATA_LOGGING && (toggle_transmission>0)){
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
            uart_putc(',');
            uart_putf(motors.totalDist);
            uart_putc('\n');
        }
        while(clockTime()<0.01){}
        if(clockTime()>0.012){
            uart_puts("ERROR: ");
            uart_putf(clockTime());
            uart_putc('\n');
        }
        counter++;
        if(counter>1000){
            motors.updateBatteryLvl();
            counter=0;
        }
        longDt+=dt;

    }
    return 0;
}

/**
 *\brief Accepts commands from controller and resolves them
 *\param[in] command Pointer to the command received
 *\param[in] pid pointer to the pid class instance used in main
 *\param[in] motorsC pointer to the motor class instance used in main
 *\param[in] mpu pointer to the mpu6050 class instance used in main
 *\return 0
 */

uint8_t resolveCommand(uint8_t *movementBuffer,uint8_t *command, uint8_t *controlMovement, PID *pid, MotorControl *motorsC, MPU *mpu){
    uint8_t tmp;
    float timer;
    switch(*command){
        case CONTROL_INFO:
            timer = clockTime();
            float newSpeed;
            newSpeed = map(movementBuffer[0],1,100,-8,8)-0.28;
            newSpeed = constrain(newSpeed,-5,5);
            float newSteering;
            newSteering = map(movementBuffer[1],1,100,-8,8)-0.28;
            timer = clockTime()-timer;
            uart_putf(timer);
            uart_putc('\n');
            motorsC->desiredSpeed = (0.95*motorsC->desiredSpeed + 0.05*newSpeed);

            if((motorsC->desiredSpeed<2) && (motorsC->desiredSpeed>-2) && (motorsC->averageSpeed<2) && (motorsC->averageSpeed>-2)){
                if(newSteering>4){
                    motorsC->SetDIR(1,'A');
                    motorsC->SetDIR(-1,'B');
                    OCR1A = 196;
                    OCR1B = 190;
                }
                else if(newSteering<-4){
                    motorsC->SetDIR(-1,'A');
                    motorsC->SetDIR(1,'B');
                    OCR1A = 196;
                    OCR1B = 190;
                }
            }
            else {
                motorsC->motorSpeedOffset = 0.7*motorsC->motorSpeedOffset + 0.3*newSteering;
            }
            *controlMovement = 0;
            motorsC->totalDist = 0;
            break;
        case REQ_BATTERY_LVL:
            uart3_putc(motorsC->getBatteryLvl());
            break;
        case REQ_TILT_ANGLE:
            int16_t angle;
            angle = (int16_t)(mpu->compXAngle*100.0);
            tmp = (angle>>8);
            uart3_putc(tmp);
            tmp = (angle&LSB);
            uart3_putc(tmp);
            break;
        case REQ_SPEED:
            int16_t speed;
            speed = (int16_t) (motorsC->averageSpeed*100.0);
            tmp = speed>>8;
            uart3_putc(tmp);
            tmp = (speed&LSB);
            uart3_putc(tmp);
            break;

        case REQ_MPU_CALIBRATION:
            mpu->calibrate(1000);
            uart3_putc('\n');
            _delay_ms(500);
            BUZZER_ON;
            _delay_ms(500);
            BUZZER_OFF;
            _delay_ms(3000);
            mpu->reset();
            break;
        case TOGGLE_HOLD:
            if(*controlMovement)*controlMovement = 0;
            else *controlMovement = 1;
            break;
    }
        *command = 0x00;
        return 0;
    }

