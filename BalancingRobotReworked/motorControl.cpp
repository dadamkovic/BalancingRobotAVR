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
    uint8_t ctrlx1;
    uint8_t ctrlx0;
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
*FUNCTION : initializes timer0A for fast PWM on pins PB5, PB6
*OUTPUT : 0
*/
uint8_t MotorDrive::initMotors(){

    DDRB |= (_BV(PD6) | _BV(PD5)) ;         // PD6, PD5 is now an output
    OCR1A = 0;                                  //motors initialized to 0V - shutdown
    OCR1B = 0;

    TCCR1A |= _BV(COM1A1);
    TCCR1A &= ~_BV(COM1A0);                 // set none-inverting mode channel A
    TCCR1A |= _BV(COM1B1);
    TCCR1A &= ~_BV(COM1B0);                 // set none-inverting mode channel B

    TCCR1A |= _BV(WGM10);
    TCCR1B |= _BV(WGM12);
    TCCR1A &= ~(_BV(WGM11));                // set fast PWM Mode - 8 bit mode

    TCCR1B |= _BV(CS11);
    TCCR1B &= ~(_BV(CS10)|_BV(CS12));      //prescaler 8, f = 7,8kHz

    SetDIR(1,'A');                              //both motors forward
    SetDIR(1,'B');

    return 0;
}




/*
*INPUT : 2 X speed of motor in percents (0 - 100)
*FUNCTION : sets the speed in timer compare registers
*OUTPUT : none
*/
void MotorDrive::SetSpeedBoth(int8_t speed){
    float calcSpeed = speed*2.55;
    if(speed>0){
         SetDIR(1,'A');
         SetDIR(1,'B');
      }
      else{
         SetDIR(-1,'A');
         SetDIR(-1,'B');
         speed = speed*-1;
      }


    loop_until_bit_is_set(TIFR1,TOV1);
    if(speed > 100){
        OCR1A = 255;
        OCR1B = 255;
    }
    else{
        OCR1A = uint8_t(calcSpeed);          //conversion from 0-100 to 0-255
        OCR1B = OCR1A;
    }
}


//TODO: maybe rework these functions bellow
/*
*INPUT : speed of motor in percents (0 - 100)
*FUNCTION : sets the speed in timer compare register for A motor
*OUTPUT : 0
*/
uint8_t MotorDrive::SetSpeedA(uint8_t speed){
  loop_until_bit_is_set(TIFR1,TOV1);
    if(speed > 100)OCR1A = 255;
    else OCR1A = ((5*speed)>>1);
    return 0;
}

/*
*INPUT : speed of motor in percents (0 - 100)
*FUNCTION : sets the speed in timer compare register for B motor
*OUTPUT : 0
*/
uint8_t MotorDrive::SetSpeedB(uint8_t speed){
  loop_until_bit_is_set(TIFR1,TOV1);
    if(speed > 100)OCR1B = 255;
    else OCR1B = ((5*speed)>>1);
    return 0;
}







