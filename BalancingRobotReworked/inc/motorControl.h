/**
 * \file motorControl.h
 * \author Daniel Adamkovic
 * \date 22/1/2019
 * \brief Declaration of the MotorControl class.
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <avr/io.h>
void encodersInit();
void initServo();
uint8_t setServoAngle(float);

/**
 * \brief Tuned to acomodate for the differences between controlled motors.
 */
#define MOTOR_A_SPEED_OFFSET 4
#define MOTOR_B_SPEED_OFFSET 1

/**
 * \brief Class controlling and monitoring two motors through an interface of H-bridge and encoder outputs.
 *
 * Aside from methods that handle interaction with the H-bridge, class also includes state variables. These are used to keep track
 * of properties like motor speed, total traveled distance, ...
 *
 * \note User is free to chose which pins will be connected to which motor inputs, but EN pins have to be connected to PD5, PD6.
 */
class MotorControl {
    public:


        MotorControl();
        MotorControl(volatile uint8_t* a_ddr,volatile uint8_t* b_ddr, volatile uint8_t* a_port,volatile uint8_t* b_port, uint8_t a_pin_1, \
                        uint8_t a_pin_2, uint8_t b_pin_1, uint8_t b_pin_2);

        ~MotorControl(){
        }
        void initInterface(volatile uint8_t* a_ddr,volatile uint8_t* b_ddr, volatile uint8_t* a_port,volatile uint8_t* b_port, uint8_t a_pin_1, \
                                   uint8_t a_pin_2, uint8_t b_pin_1, uint8_t b_pin_2);

        void SetSpeedBoth(int8_t);
        void setSpeedIndividually(int8_t);
        uint8_t initMotors();
        uint8_t getBatteryLvl();
        uint8_t updateBatteryLvl();
        float measureCurrent();
        float getCurrent();

        volatile float desiredSpeed = 0;
        volatile float motorSpeedOffset=0;
        volatile float totalDist = 0;           ///< Used to keep track of the total distance traveled
        volatile float speedAB,speedCD = 0;     ///< Used to keep track of the speed of each wheel
        volatile int32_t encoderAB,encoderCD = 0; ///< Used to keep track of the total number of detected signal edges
        volatile float oldSpeed = 0;            ///< Necessary to know previous speed to implement filter
        volatile float averageSpeed = 0;        ///< Average speed of both wheels
        volatile float desiredDistance = 0;
        volatile float current=0.0;
        uint8_t currBattLvl = 50;
        uint8_t commandDecay = 1;
        uint8_t SetDIR(int8_t, char);
    private:

        uint8_t AddOffset(uint8_t, int8_t);
        volatile uint8_t *_Motor_A_DDR;         ///< Holds the address of the DDRx register for both, IN inputs of a H-bridge A
        volatile uint8_t *_Motor_B_DDR;         ///< Holds the address of the DDRx register for both, IN inputs of a H-bridge B
        volatile uint8_t *_Motor_A_PORT;        ///< Holds the address of the PORTx register for both, IN inputs of a H-bridge A
        volatile uint8_t *_Motor_B_PORT;        ///< Holds the address of the PORTx register for both, IN inputs of a H-bridge B
        uint8_t _Motor_A_PIN_1;                 ///< Pin number of the IN1 input of H-bridge A
        uint8_t _Motor_A_PIN_2;                 ///< Pin number of the IN2 input of H-bridge A
        uint8_t _Motor_B_PIN_1;                 ///< Pin number of the IN1 input of H-bridge B
        uint8_t _Motor_B_PIN_2;                 ///< Pin number of the IN2 input of H-bridge B
        volatile float current_sensor_resistance = 0.3;


};
#endif /* MOTORCONTROL_H_ */
