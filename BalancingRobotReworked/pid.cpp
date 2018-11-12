/*
 * pid.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: daniel
 */


#include "pid.h"

float PID::giveOutput(float input, float target, float dt, uint16_t constrainI=0){
    float output  = 0;
    float error = input-target;
    error_integral += error;
    if(constrainI){
        error_integral = constrain(error_integral,-constrainI,constrainI);
    };
    error_derivative = (Tf*error_derivative + (error - old_error))/(dt + Tf);       //implements filtering constant Tf
    output = P*(error)+I*(error_integral)*dt+D*error_derivative;

    old_error = error;
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
