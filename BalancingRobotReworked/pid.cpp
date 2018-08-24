/*
 * pid.cpp
 *
 *  Created on: Aug 24, 2018
 *      Author: daniel
 */


#include "pid.h"

float PID::giveOutput(double input, double target, double dt){
    float output  = 0;
    float error = target - input;
    output = P*(error)+I*(error_sum+error)*dt+D*(error-old_error)/dt;
    error_sum += error;
    old_error = error;
    return output;
}

void PID::changeP(float newP){
  P = newP;
}


