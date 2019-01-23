/**
 * \file pid.cpp
 * \author Daniel Adamkovic
 * \date 22/1/2019
 *
 * \brief Contains definitions of PID class methods.
 */


#include "pid.h"

#define MAX_ADC_VAL 1024.0        //might be lower if input voltage <2.56


/**
 * \brief Calculates the output of PID, given input parameters.
 * \param[in] input Value that corresponds to the level of current measurement of some state. E.g.: Current angle.
 * \param[in] target Desired value of the state we care about. E.g.: Angle we want to achieve.
 * \param[in] dt The time elapsed between measurements of real state values, measured in seconds.
 * \param[in] constrainI If set, determines the maximal possible accumulated value of integral error. If set to 0 than error is unlimited.
 * \return Output value of PID regulator.
 *
 * Instead of a simple PID regulator, we chose to implement filtering of the D error term. This is set through the PID class' internal variable
 * Tf. This way the inherent discreetness of the measured values doesn't cause D error term to change erraticaly.
 */
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


/**
 * \brief Changes the P parameter of the regulator.
 * \param[in] newP New value of the parameter.
 * \return void
 */
void PID::changeP(float newP){
  P = newP;
}

/**
 * \brief Changes the I parameter of the regulator.
 * \param[in] newI New value of the parameter.
 * \return void
 */
void PID::changeI(float newI){
  I = newI;
}

/**
 * \brief Changes the D parameter of the regulator.
 * \param[in] newD New value of the parameter.
 * \return void
 */
void PID::changeD(float newD){
  D = newD;
}

/**
 * \brief Simplifies the tuning of the PID regulator's parameters.
 * \param[in] select Either 'P', 'I' or 'D', depending on which parameter we want to tune.
 * \param[in] maxParamValue Maximal value that tuned parameter can be set to.
 * \return New value of the parameter.
 *
 * This function takes care of initializing ADC converter, used for sampling the voltage set with potentiometer.
 * the read value is scaled based on the maximal allowed value for a given parameter, which is then set accordingly.
 *
 * \todo Make the maximal possible levels of parameter defined values
 */
float PID::tunePID(char select, float maxParamValue){
    uint16_t ADCval;
    ADMUX |= _BV(REFS0) | _BV(REFS1);                                   //2.56 V reference
    ADMUX &= ~(_BV(MUX0)|_BV(MUX1)|_BV(MUX2)|_BV(MUX3)|_BV(MUX4));      //Channel ADC0
    ADCSRA |= _BV(ADEN);                                                //ADC initialized
    ADCSRA |= _BV(ADSC);                                                // starts first conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);                               //bit clear when read is complete
    ADCval = ADC;
    float param = 0;
    switch(select){
        case 'P':
        {
            P = (ADCval/MAX_ADC_VAL)*maxParamValue;
            param = P;
            break;
        }
        case 'I':
        {
            I = (ADCval/MAX_ADC_VAL)*maxParamValue;
            param = I;
            break;
        }
        case 'D':
        {
            D = (ADCval/MAX_ADC_VAL)*maxParamValue;
            param = D;
            break;
        }
    }
    return param;
}

