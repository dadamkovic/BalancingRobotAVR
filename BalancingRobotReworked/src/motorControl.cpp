/**
 * \file motorControl.cpp
 * \author Daniel Adamkovic
 * \date 22/1/2019
 *
 * \brief Definition of methods used by MotorControl class.
 */


#include "motorControl.h"


/**
 * \brief Constructor of the MotorControl class
 *
 * \param[in] a_ddr Address of the DDRx register to which H-bridge A inputs are connected.
 * \param[in] b_ddr Address of the DDRx register to which H-bridge B inputs are connected.
 * \param[in] a_port Address of the PORTx register to which H-bridge A inputs are connected.
 * \param[in] b_port Address of the PORTx register to which H-bridge B inputs are connected.
 * \param[in] a_pin_1 Number of the pin to which the H-bridge's A IN1 input is connected.
 * \param[in] a_pin_2 Number of the pin to which the H-bridge's A IN2 input is connected.
 * \param[in] b_pin_1 Number of the pin to which the H-bridge's B IN1 input is connected.
 * \param[in] b_pin_2 Number of the pin to which the H-bridge's B IN2 input is connected.
 *
 * User can choose how to wire the H-bridge to the Arduino Mega board, but the choice is not completely arbitrary.
 * User has to wire both inputs IN1/IN2 of a given H-bridge to the Arduino pins that are controlled by the same DDRx register.
 * EN input has to be wired to PD5 for H-bridge A and PD6 for H-bridge B.
 * The class is able to control practically any DC motors connected to an H-bridge
 * which interface consists of IN1/IN2 inputs for direction control and EN input for enabling power to motor.
 * EN pin to
 */
MotorControl::MotorControl(volatile uint8_t* a_ddr,volatile uint8_t* b_ddr, volatile uint8_t* a_port,volatile uint8_t* b_port, uint8_t a_pin_1, \
                           uint8_t a_pin_2, uint8_t b_pin_1, uint8_t b_pin_2)
        : _Motor_A_DDR(a_ddr),_Motor_B_DDR(b_ddr), _Motor_A_PORT(a_port), _Motor_B_PORT(b_port), _Motor_A_PIN_1(a_pin_1), _Motor_A_PIN_2(a_pin_2),\
          _Motor_B_PIN_1(b_pin_1), _Motor_B_PIN_2(b_pin_2) {}

/**
 * \brief Initializes motors, only called <b> once </b> in a program run.
 * \return 0
 *
 * The method initializes Timer 1 into non-inverting fast PWM mode and sets the corresponding
 * pins to be outputs changing based on Timer 1 control registers setup.
 */
void MotorControl::initMotors(){

    DDRB |= (_BV(PB6) | _BV(PB5)) ;             // PD6, PD5 is output
    OCR1A = 0;                                  //motors initialized to 0V - shutdown
    OCR1B = 0;

    TCCR1A |= _BV(COM1A1);
    TCCR1A &= ~_BV(COM1A0);                     // set none-inverting mode channel A
    TCCR1A |= _BV(COM1B1);
    TCCR1A &= ~_BV(COM1B0);                     // set none-inverting mode channel B

    TCCR1A |= _BV(WGM10);
    TCCR1B |= _BV(WGM12);
    TCCR1A &= ~(_BV(WGM11));                    // set fast PWM Mode - 8 bit mode

    TCCR1B |= _BV(CS11);
    TCCR1B &= ~(_BV(CS10)|_BV(CS12));           //prescaler 8


    *_Motor_A_DDR |= (_BV(_Motor_A_PIN_1)|(_BV(_Motor_A_PIN_2)));
    *_Motor_B_DDR |= (_BV(_Motor_B_PIN_1)|(_BV(_Motor_B_PIN_2)));
    SetDIR(1,'A');                              //both motors forward
    SetDIR(1,'B');
}

/**
 * brief Sets the direction of motor turning.
 * \param[in] dir Any positive or negative value, depending on the desired direction (only sign matters).
 * \param[in] motor Either character 'A' or 'B' depending on which motor the user wants to control.
 * \return 0
 */
uint8_t MotorControl::SetDIR(int8_t dir, char motor){
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

    if(dir>0){
        *motor_port &= ~(1 << ctrlx0);
        *motor_port |= (1 << ctrlx1);
    }
    else{
        *motor_port |= (1 << ctrlx0);
        *motor_port &= ~(1<< ctrlx1);
    }
    return 0;
}

/**
 * \brief sets the PWM for the H-bridge, taking into account the individual offsets for each motor
 * \param[in] motorSpeed value between -100 and 100 where negative values indicate reversed direction
 */
void MotorControl::setSpeedIndividually(int8_t motorSpeed){
    int8_t motorSpeedA = motorSpeed + (int8_t)motorSpeedOffset;
    int8_t motorSpeedB = motorSpeed - (int8_t) motorSpeedOffset;
    if(motorSpeedA>0){
         SetDIR(1,'A');
         motorSpeedA += MOTOR_A_SPEED_OFFSET;
    }
    else{
         SetDIR(-1,'A');
         motorSpeedA = motorSpeedA*-1;
         motorSpeedA += MOTOR_A_SPEED_OFFSET;
    }
    if(motorSpeedB>0){
        SetDIR(1,'B');
        motorSpeedB += MOTOR_B_SPEED_OFFSET;
       }
   else{
        SetDIR(-1,'B');
        motorSpeedB = motorSpeedB*-1;
        motorSpeedB += MOTOR_B_SPEED_OFFSET;
   }
    uint8_t setSpeedA = (motorSpeedA*2)+50;   //conversion from 0-100 to 0-255
    uint8_t setSpeedB = (motorSpeedB*2)+50;   //conversion from 0-100 to 0-255
    if(motorSpeedA >= 100){
        OCR1A = 255;
    }
    else{
        OCR1A = setSpeedA;
    }
    if(motorSpeedB >= 100){
        OCR1B = 255;
    }
    else{
        OCR1B = setSpeedB;
    }
}


/**
 * \brief Private helper function, that makes sure adding offset deosn't cause overflow or underflow.
 * \param[in] value Value that we want to add offset to.
 * \param[in] offset Offset we wish to add.
 * \return Value with offset correctly added.
 *
 * Without this function we could end up causing overflow or underflow. E.g.:If we were trying to add offset of 10
 * to value 250 we would end up with result 5, which is obviously wrong. In this case the function would return the result
 * of 255 which is the highest number we could possibly represent.
 */
uint8_t MotorControl::AddOffset(uint8_t value, int8_t offset){
    if(((value+offset)<offset)&&(offset>0))return 255;
    else if(((value+offset)>(255+offset))&&(offset<0))return 0;
    else return (value+offset);
}

/**
 * \brief sets up the ADC and reads the voltage value from the battery
 * The method updates the class parameter currBattLvl with new value based on the voltage read from
 * voltage divider connected to the battery leads. If the value falls under a given threshold
 * a red LED light is turned on to signal to the user the need for battery recharge.
 */
uint8_t MotorControl::updateBatteryLvl(){
    ADMUX |= _BV(REFS0);
    ADMUX &= ~(_BV(REFS1));                                             //5V reference
    ADMUX &= ~(_BV(MUX0)|_BV(MUX1)|_BV(MUX2)|_BV(MUX3)|_BV(MUX4));      //Channel ADC0
    ADCSRA |= _BV(ADEN);                                                //ADC initialized
    ADCSRA |= _BV(ADSC);                                                // starts first conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);                               //bit clear when read is complete
    uint16_t result;
    result = ADCL;
    result |= ADCH<<8;
    //3.77
    volatile float tmp = (map((((float)result*3.77)/1024.0),3.5,3.77,0,100));
    tmp = (float)currBattLvl*0.9 + 0.1*tmp;
    currBattLvl = (uint8_t)tmp;

    if(currBattLvl<30){
        ROBOT_BATTERY_ON;
    }
    else {
        ROBOT_BATTERY_OFF;
    }
    return currBattLvl;
}

/**
 * \brief returns current measured battery lvl
 */
uint8_t MotorControl::getBatteryLvl(){
    return currBattLvl;
}

/*
float MotorControl::measureCurrent(){
    ADMUX |= _BV(REFS0);
    ADMUX &= ~(_BV(REFS1));             //5V reference
    ADMUX |= _BV(MUX0);                //ADC3 - pos, ADC2 - neg, 200 x amplification
    ADMUX &= ~(_BV(MUX2)|_BV(MUX3)|_BV(MUX4)|_BV(MUX1));
    ADCSRA |= _BV(ADEN);                                                //ADC initialized
    ADCSRA |= _BV(ADSC);                                                // starts first conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);                               //bit clear when read is complete
    uint16_t result;
    result = ADCL;
    result |= ADCH<<8;
    volatile float voltage_lvl = ((float)result*4.37)/(1024.0);
    current = voltage_lvl/currentSensorResistance;
    return current;
}
*/

/**
 * \brief returns current measured battery lvl
 */
float MotorControl::getCurrent(){
    return current;
    //return (float) (ADCval);
}



