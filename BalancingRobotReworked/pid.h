
/*
 * pid.h
 *
 * Created: 3.8.2018 14:02:54
 *  Author: Asus
 */


#ifndef PID_H_
#define PID_H_
#include "inttypes.h"
class PID{
    public:
        PID(float proportional, float integrate, float derivative):
        P(proportional), I(integrate),D(derivative){};

        float P = 1;
        float I = 1;
        float D = 1;
        float Tf = 30;
        float old_error=0;
        float error_integral = 0;
        float error_derivative = 0;
        float giveOutput(float,float,float,float);
        void changeP(float);
        void changeI(float);
        void changeD(float);
    };

/*
 * INPUT : x = value to constrain, minValue - maxValue = constrain borders
 * OUTPUT :
 */

static float constrain(float x, float minValue, float maxValue){
    if(x < minValue)return minValue;
    else if(x > maxValue)return maxValue;
    else return x;
}


#endif /* PID_H_ */
