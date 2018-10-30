/*
 * timeTracking.h
 *
 *  Created on: Sep 6, 2018
 *      Author: daniel
 */

#ifndef TIMETRACKING_H_
#define TIMETRACKING_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "inttypes.h"

void clockInit();
void clockStart();
float clockTime();
void clockReset();
void clockShutdown();

void encoderClockInit();
void encoderClockStart();
void encoderClockStop();

extern volatile int16_t encoderAB;
extern volatile float speedAB, speedCD;


#endif /* TIMETRACKING_H_ */
