
/*
 * pid.h
 *
 * Created: 3.8.2018 14:02:54
 *  Author: Asus
 */


#ifndef PID_H_
#define PID_H_

class PID{
    public:
        PID(float proportional, float integrate, float derivative):
        P(proportional), I(integrate),D(derivative){};

        float P = 1;
        float I = 1;
        float D = 1;
        float old_error=0;
        float error_sum = 0;
        float giveOutput(double,double,double);
        void changeP(float);
    };




#endif /* PID_H_ */