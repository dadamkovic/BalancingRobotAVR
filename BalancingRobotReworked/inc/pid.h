/**
 * \file pid.h
 * \author Daniel Adamkovic
 * \brief Declaration of PID class and its methods and prarameters
 */


#ifndef PID_H_
#define PID_H_
#include "inttypes.h"
#include <avr/io.h>

extern float constrain(float, float, float);
/**
 * \brief Class that holds all information and methods necessary to implement PID regulator.
 */
class PID{
    public:
        PID(float proportional, float integrate, float derivative):
        P(proportional), I(integrate),D(derivative){};
        ~PID(){};

        float P = 1;
        float I = 1;
        float D = 1;
        float Tf = 0.6;
        float old_error=0;
        float error_integral = 0;
        float error_derivative = 0;
        float giveOutput(float,float,float,float);
        void changeP(float);
        void changeI(float);
        void changeD(float);
        float tunePID(char, float);
        void addIntErr(float);
    };



#endif /* PID_H_ */
