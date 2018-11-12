/*
 * MotorControl.h
 *
 * Created: 18.7.2018 13:17:43
 *  Author: Asus
 * Motor ENABLE1,ENABLE2 pins are connected to PD5,PD6 respectively the rest are free for the user to chose
 */


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


class MotorDrive {
    public:
        float Motor_SPEED[2];

        MotorDrive(volatile uint8_t* a_port,volatile uint8_t* b_port, uint8_t a_pin_1, uint8_t a_pin_2, uint8_t b_pin_1, uint8_t b_pin_2)
        : _Motor_A_PORT(a_port), _Motor_B_PORT(b_port), _Motor_A_PIN_1(a_pin_1), _Motor_A_PIN_2(a_pin_2),
        _Motor_B_PIN_1(b_pin_1), _Motor_B_PIN_2(b_pin_2) {

        }
        ~MotorDrive(){
        }


        uint8_t SetDIR(int8_t, char);
        void SetSpeedBoth(int8_t);
        uint8_t SetSpeedA(uint8_t);
        uint8_t SetSpeedB(uint8_t);
        uint8_t initMotors();

        volatile float speedAB,speedCD;
        volatile int16_t encoderAB, encoderCD;

    private:
        volatile uint8_t *_Motor_A_PORT;
        volatile uint8_t *_Motor_B_PORT;
        uint8_t _Motor_A_PIN_1;
        uint8_t _Motor_A_PIN_2;
        uint8_t _Motor_B_PIN_1;
        uint8_t _Motor_B_PIN_2;


};







#endif /* MOTORCONTROL_H_ */
