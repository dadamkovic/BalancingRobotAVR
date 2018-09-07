/*
 * timeTracking.h
 *
 *  Created on: Sep 6, 2018
 *      Author: daniel
 */

#ifndef TIMETRACKING_H_
#define TIMETRACKING_H_

#include <avr/io.h>
#include "inttypes.h"

void clockInit();
void clockStart();
double clockTime();
void clockReset();
void clockShutdown();



#endif /* TIMETRACKING_H_ */
