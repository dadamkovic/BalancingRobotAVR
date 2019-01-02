/*
 * pid.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: daniel
 */


#include "pid.h"

float PID::giveOutput(float input, float target, float dt, float constrainI){
    float output  = 0;
    float Perror = target-input;
    error_integral += Perror;
    if(constrainI){
        error_integral = constrain(error_integral,-constrainI,constrainI);
    };
    error_derivative = (Tf*error_derivative + (Perror - old_error))/(dt + Tf);       //implements filtering constant Tf
    output = P*(Perror)+I*(error_integral)*dt+D*error_derivative;

    old_error = Perror;
    return output;
}

void PID::changeP(float newP){
  P = newP;
}

void PID::changeI(float newI){
  I = newI;
}

void PID::changeD(float newD){
  D = newD;
}

float PID::tunePID(char select){
    uint16_t ADCval;
    ADCSRA |= _BV(ADSC); // starts first conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);
    ADCval = ADC;
    float param;
    switch(select){
        case 'P':
        {
            P = (ADCval/1000.0);
            param = P;
            break;
        }
        case 'I':
        {
            I = (ADCval/100.0);
            //error_integral = 0;
            param = I;
            break;
        }
        case 'D':
        {
            D = (ADCval/5000.0);
            param = D;
            break;
        }
    }
    return param;
}

void PID::addIntErr(float err){
    error_integral = err;
}
