/*
 * motorControl.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: daniel
 */


#include "motorControl.h"



/*
*INPUT : direction of motor (<0 BACK, >0 FORWARD), char (A, B) selects which motor to control
*FUNCTION : sets PINS based on the desired direction of spin
*OUTPUT : 0
*/
uint8_t MotorDrive::SetDIR(uint8_t dir, char motor){
    volatile uint8_t *motor_port;
    int ctrlx1 = 0;
    int ctrlx0 = 0;
    switch(motor){
        case 'A':
                motor_port = _Motor_A_PORT;
                ctrlx0 = _Motor_A_PIN_1;
                ctrlx1 = _Motor_A_PIN_2;
                break;
        case 'B':
                motor_port = _Motor_B_PORT;
                ctrlx0 = _Motor_B_PIN_1;
                ctrlx1 = _Motor_B_PIN_2;
                break;
        default:
                return 1;
                break;
    }

    if(dir<0){
        *motor_port &= ~(1 << ctrlx0);
        *motor_port |= (1 << ctrlx1);
    }
    else{
        *motor_port |= (1 << ctrlx0);
        *motor_port &= ~(1<< ctrlx1);
    }

   return 0;
}

/*
*INPUT : void
*FUNCTION : initializes timer0A for fast PWM on pins PD5, PD6 (5,6 on Arduino)
*OUTPUT : 0
*/
uint8_t MotorDrive::initMotors(){

    DDRD |= (1 << DDD6) | (1 << DDD5) ;         // PD6, PD5 is now an output
    OCR0A = 0;                                  //motors initialized to 0V - shutdown
    OCR0B = 0;

    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
    // set none-inverting mode

    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    // set fast PWM Mode

    TCCR0B |=  (1 << CS00)  ;                   //prescaler frequency aproximately 100Hz
    //TCCR0B |=  (1 << CS02)  ;
    SetDIR(1,'A');                              //both motors forward
    SetDIR(1,'B');

    DDRD &= ~(1 << ENCODER0) | ~(1 << ENCODER1);
    PORTD &= ~(1 << ENCODER0) | ~(1 << ENCODER1);

    EICRA |= (1 << ISC10) | (1 << ISC11);       //on rising edge interrupt PD2
    EICRA |= (1 << ISC00) | (1 << ISC01);       //on rising edge interrupt PD3
    EIMSK |= (1 << INT0) | (1 << INT1);         //enables both interrupts

    return 0;
}

float* MotorDrive::GetSpeed(){
    uint8_t old_encoder_0 = encoder0;
    uint8_t old_encoder_1 = encoder1;
    _delay_ms(TIME_OF_SAMPLE);
    Motor_SPEED[0] = (((float)encoder0 - (float)old_encoder_0)/ENCODER_STATES)*(2*3.14);        //angle turned by the motor
    Motor_SPEED[0] = Motor_SPEED[0]*0.00335;                                                    //multiplication by radius of the wheel 3.35cm
    Motor_SPEED[0] = Motor_SPEED[0]/TIME_OF_SAMPLE;                                             //v = s/t; we get speed forward
    Motor_SPEED[1] = (((float)encoder1 - (float)old_encoder_1)/ENCODER_STATES)*(2*3.14);
    Motor_SPEED[1] = Motor_SPEED[1]*0.00335;
    Motor_SPEED[1] = Motor_SPEED[1]/TIME_OF_SAMPLE;
    return Motor_SPEED;
    }


/*
*ISR which takes care of encoder0
*/

ISR (INT0_vect)
{
    static uint8_t encoder0=0;
    encoder0+=1;                            //adds impulz
    if(encoder0>32700)encoder0=0;               //zeroes it so we can later measure speed
}


ISR (INT1_vect)
{
    static uint8_t encoder1=0;
    encoder1+=1;
    if(encoder1>32700)encoder1=0;
}

/*
*INPUT : 2 X speed of motor in percents (0 - 100)
*FUNCTION : sets the speed in timer compare registers
*OUTPUT : none
*/
void MotorDrive::SetSpeedBoth(int8_t speed){
    if(speed>0){
         SetDIR(1,'A');
         SetDIR(1,'B');
      }
      else{
         SetDIR(-1,'A');
         SetDIR(-1,'B');
         speed = speed*-1;
      }


    loop_until_bit_is_set(TIFR0,TOV0);
    if(speed > 100){
        OCR0A = 215;
        OCR0B = 215;
    }
    else{
        OCR0A = ((4*speed)>>1)+15;          //conversion from 0-100 to 0-215
        OCR0B = OCR0A;
    }
}


//TODO: maybe rework these functions bellow
/*
*INPUT : speed of motor in percents (0 - 100)
*FUNCTION : sets the speed in timer compare register for A motor
*OUTPUT : 0
*/
uint8_t MotorDrive::SetSpeedA(uint8_t speed){
  loop_until_bit_is_set(TIFR0,TOV0);
    if(speed > 100)OCR0A = 255;
    else OCR0A = ((5*speed)>>1);
    return 0;
}

/*
*INPUT : speed of motor in percents (0 - 100)
*FUNCTION : sets the speed in timer compare register for B motor
*OUTPUT : 0
*/
uint8_t MotorDrive::SetSpeedB(uint8_t speed){
  loop_until_bit_is_set(TIFR0,TOV0);
    if(speed > 100)OCR0B = 255;
    else OCR0B = ((5*speed)>>1);
    return 0;
}







