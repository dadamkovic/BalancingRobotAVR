/**
 * \file timeTracking.h
 */

#ifndef TIMETRACKING_H_
#define TIMETRACKING_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "inttypes.h"

#define TIMER5_SIZE 65535.0       //< Maximal size of Timer 5
#define TIMER5_PERIOD 0.032768  //< Current period of Timer 5

void clockInit();
void clockStart();
float clockTime();
void clockReset();
void clockShutdown();

void encoderClockInit();
void encoderClockStart();
void encoderClockStop();



#endif /* TIMETRACKING_H_ */
