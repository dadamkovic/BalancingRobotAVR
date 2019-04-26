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


#define DEBUG_OUTPUT 0             //set to 0 to disable debugging info
#define DATA_LOGGING 0


float EEMEM xCalAddr;       ///Pointers to memory holding calibration values
float EEMEM yCalAddr;
float EEMEM zCalAddr;

uint8_t resolveCommand(uint8_t *movementBuffer,uint8_t *command, uint8_t *controlMovement, PID *pid, MotorControl *motorsC, MPU *mpu);

MotorControl motors(&DDRC,&DDRC,&PORTC,&PORTC,0,2,4,6);       //motors controlled by PB1,PB2,PB3,PB4

int main(void){
    ROBOT_LED_ON;           //turns on status LED (green)
    float dt,longDt = 0;
    uint16_t counter = 0;
    float motorPower = 0;
    float desiredAngle = 0;
    uint8_t bufferIndex = 0;
    uint8_t commandBuffer[2] = {0,0};       //buffers received commands from controller
    uint8_t command = 0x00;                 //default command
    int8_t toggleTransmission = -1;        //used to stop transmitting when PC closes port
    uint8_t trackPosition = 1;            //when set to 0 robot doesn't track position


    /*All  necessary initializations*/
    sei();
    motors.initMotors();
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
    uart3_init(UART_BAUD_SELECT(57600, F_CPU));
    initEncoders();
    initServo();
    clockInit();
    while(initIIC()!=0);        //

    BUZZER_INIT;
    BUZZER_ON;


/*
 *       motorSpeed           compXAngle
 *          |                     |
 *          |                     |
 *
 * ------------>speedAnglePID----->anglePwmPID
 */


    clockStart();
    MPU mpu6050;

    dt = clockTime();   //gets initial dt

    setServoAngle(SERVO_OFFSET);
    clockReset();
    _delay_ms(10);

    //PID speedAnglePID(0.55,0.05,0.02);                  //0.69,0.03,0.02
    PID speedAnglePID(0.55,0.3,0.03);
    //PID speedAnglePID(0,0.00,0);
    //PID anglePwmPID(16,0.0,0.66);                        //38,0.24
    PID anglePwmPID(14.0,0.0,0.56);
    //PIC distancePID(15,0,0);
    PID distancePID(10,10,0);
    //PID servoPID(0,4.5,0);
    PID servoPID(10,0,0);
    //float oldServo = mpu6050.compYAngle*RAD_TO_DEG;
    BUZZER_OFF;
    while(1){
        dt = clockTime();
        clockReset();
        mpu6050.updateValues(dt);
        if(longDt > 0.05){                                      //every 50ms recompute PID output
            float currSpeed = motors.averageSpeed;
            float desiredSpeed = motors.desiredSpeed;
            if(trackPosition){
                desiredSpeed = constrain(distancePID.giveOutput(motors.totalDist,0,longDt,1),-5,5);
            }
            desiredAngle = speedAnglePID.giveOutput(currSpeed,desiredSpeed,longDt,10);
            desiredAngle = constrain(desiredAngle,-9,9);
            /*float servoAngle = servoPID.giveOutput(mpu6050.compYAngle*RAD_TO_DEG,0,longDt,150);
            servoAngle = constrain(servoAngle,-50,50);
            oldServo = 0.8*oldServo + 0.2*servoAngle;
            setServoAngle(oldServo+SERVO_OFFSET);*/
            longDt = 0;
        }

        motorPower= anglePwmPID.giveOutput(mpu6050.compXAngle*RAD_TO_DEG,desiredAngle,dt,0);        //calling PID to give us value for motors
        motorPower = constrain(motorPower,-90,90);                                                  //constraining PID output

        //setServoAngle(SERVO_OFFSET);

        if(((mpu6050.compXAngle*RAD_TO_DEG)>30) || ((mpu6050.compXAngle*RAD_TO_DEG)<-30)){          //checks if it hasn't fallen
            motors.setSpeedIndividually(0);
        }
        else{
            motors.setSpeedIndividually((int8_t)motorPower);
        }

        motors.motorSpeedOffset = (0.9999*motors.motorSpeedOffset);         //check if these fit later!!!!!!!!!!!!!!!
        motors.desiredSpeed = (0.9999*motors.desiredSpeed);

        while(clockTime()<0.005){
            if(uart_available()){
                if(uart_getc()=='X'){
                    toggleTransmission *= -1;
                }
            }
            if(uart3_available()){
                command = uart3_getc();
                /*code bellow continues filling the command buffer if movement command is expected*/
                if(bufferIndex){
                    commandBuffer[bufferIndex-1] = command;
                    if(bufferIndex == 2){
                        command = CONTROL_INFO;
                        resolveCommand(commandBuffer,&command, &trackPosition, &speedAnglePID, &motors, &mpu6050);
                        bufferIndex = 0;
                    }
                    else{
                        bufferIndex++;
                    }
                }
                if(command == CONTROL_INFO){
                    bufferIndex = 1;
                }
                else if((command != CONTROL_INFO) && (bufferIndex == 0)&&(command != 0x00)){
                    resolveCommand(commandBuffer,&command, &trackPosition, &speedAnglePID, &motors, &mpu6050);
                }
            }
        }

        if((counter == 201) && (DEBUG_OUTPUT == 1)){                            //used for printing debug stuff
            uart_putf(mpu6050.xCal);
            uart_putc('\n');
            uart_putf(dt);
            uart_putc('\n');
            counter=0;
        }

        if(((counter % 5) == 0) && DATA_LOGGING && (toggleTransmission>0)){     //used for getting graph data
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
        if(clockTime()>0.006){
            uart_puts("ERROR: ");
            uart_putf(clockTime());
            uart_putc('\n');
            dt = 0.005;
        }
        counter++;
        if((counter % 100 )== 0){
            motors.updateBatteryLvl();
        }
        if(counter>1000)counter = 0;
        longDt+=dt;
    }
    return 0;
}

/**
 *\brief Accepts commands from controller and resolves them
 *\
 *\This function is essentially a finite-state machine with the data received from controller to be taken as a state.
 *\The function handles robot movement decoding, communication with the controller, calibration and distance tracking toggling.
 *\
 *\param[in] movementBuffer pointer to the data in the buffer
 *\param[in] command pointer to the command received
 *\param[in] controlMovement pointer that allows us to turn off or on distance tracking
 *\param[in] pid pointer to the pid class instance used in main
 *\param[in] motorsHandle pointer to the motor class instance used in main
 *\param[in] mpu pointer to the mpu6050 class instance used in main
 *\return 0
 */

uint8_t resolveCommand(uint8_t *movementBuffer,uint8_t *command, uint8_t *controlMovement, PID *pid, MotorControl *motorsHandle, MPU *mpu){
    switch(*command){
        case CONTROL_INFO:
            float newSpeed;
            newSpeed = map(movementBuffer[0],1,100,-5,5)-0.18;
            //newSpeed = constrain(newSpeed,-5,5);
            float newSteering;
            newSteering = map(movementBuffer[1],1,100,-7,7)-0.2;

            /*if((motorsC->averageSpeed<2) && (motorsC->averageSpeed>-2)&&((newSteering>4)||(newSteering<-4))){
                if(newSteering<-4){
                    motorsC->SetDIR(1,'A');
                    motorsC->SetDIR(-1,'B');
                    OCR1A = 196;
                    OCR1B = 190;

                }
                else if(newSteering>4){
                    motorsC->SetDIR(-1,'A');
                    motorsC->SetDIR(1,'B');
                    OCR1A = 196;
                    OCR1B = 190;
                }
            }*/
            motorsHandle->desiredSpeed = (0.75*motorsHandle->desiredSpeed + 0.25*newSpeed);
            motorsHandle->motorSpeedOffset = 0.9*motorsHandle->motorSpeedOffset + 0.1*newSteering;

            *controlMovement = 0;
            motorsHandle->totalDist = 0;
            break;
        case REQ_BATTERY_LVL:
            uart3_putc(motorsHandle->getBatteryLvl());
            break;
        case REQ_TILT_ANGLE:
            uart3_putf(mpu->compXAngle);
            break;
        case REQ_SPEED:
            uart3_putf(motorsHandle->averageSpeed);
            break;
        case REQ_DISTANCE:
            uart3_putf(motorsHandle->totalDist);
            break;
        case REQ_CURRENT:
            uart3_putf(motorsHandle->getCurrent());
            break;
        case REQ_MPU_CALIBRATION:
            mpu->calibrate(5000);
            uart3_putc('\n');
            _delay_ms(500);
            BUZZER_ON;
            _delay_ms(500);
            BUZZER_OFF;
            _delay_ms(3000);
            motorsHandle->totalDist = 0;
            mpu->compXAngle = 0;
            break;
        case TOGGLE_HOLD:
            if(*controlMovement)*controlMovement = 0;
            else *controlMovement = 1;
            break;
    }
        *command = 0x00;
        return 0;
    }

