/*
 * utility.h
 *
 *  Created on: Feb 21, 2019
 *      Author: daniel
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

float constrain(float, float, float);
float map(float, float, float, float, float);

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
#include "inttypes.h"

class FIFOBuffer{
    public:
        FIFOBuffer(){};
        int32_t pop();
        void add(int32_t);
        uint8_t filled();
    private:
        uint8_t buffer_head = 0;
        volatile int32_t buffer_items[10];
        uint8_t buff_filled = 0;
};
#endif /* INC_UTILITY_H_ */
