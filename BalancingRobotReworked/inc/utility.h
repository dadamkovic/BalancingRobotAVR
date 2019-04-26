/*
 * utility.h
 *
 *  Created on: Feb 21, 2019
 *      Author: daniel
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

#include "inttypes.h"


#define ANGLE_OFFSET 0
#define SERVO_OFFSET -8
#define UART_BAUD_RATE 57600
#define LSB 0x00ff
#define MSB 0xff00

float constrain(float x, float minValue, float maxValue);
float map(float num2map, float botInit, float topInit, float mapLow, float mapHigh);


#define BUZZER_PIN 0
#define BUZZER_DDR DDRL
#define BUZZER_PORT PORTL
#define BUZZER_INIT BUZZER_DDR |= _BV(BUZZER_PIN)
#define BUZZER_ON BUZZER_PORT |= _BV(BUZZER_PIN)
#define BUZZER_OFF BUZZER_PORT &= ~_BV(BUZZER_PIN)
#define BUZZER_TOGGLE BUZZER_PORT ^= _BV(BUZZER_PIN)

#define ROBOT_LED_ON DDRA |= _BV(7);PORTA |= _BV(7)
#define ROBOT_BATTERY_ON DDRA |= _BV(5);PORTA |= _BV(5)
#define ROBOT_BATTERY_OFF DDRA |= _BV(5);PORTA &= ~(_BV(5))


class FIFOBuffer{
    public:
        FIFOBuffer(){};
        int32_t pop();
        void add(int32_t);
        uint8_t filled();
    private:
        uint8_t bufferHead = 0;
        volatile int32_t bufferItems[10];
        uint8_t buffFilled = 0;
};
#endif /* INC_UTILITY_H_ */
