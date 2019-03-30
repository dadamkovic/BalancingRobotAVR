/*
 * robotControl.h
 *
 *  Created on: Feb 15, 2019
 *      Author: daniel
 */

#ifndef INC_ROBOTCONTROL_H_
#define INC_ROBOTCONTROL_H_

#include "uart.h"
#include "pid.h"
#include "motorControl.h"


#define CONTROL_INFO 0xff
#define SEND_CONTROL_INFO 0x01
#define REQ_BATTERY_LVL 0x02
#define REQ_TILT_ANGLE 0x03
#define REQ_MPU_CALIBRATION 0x04
#define REQ_SPEED 0x06
#define TOGGLE_HOLD 0x07

/**
#define H_REGISTER1 &DDRC
#define H_REGISTER2 &DDRC
#define H_PORT1 &PORTC
#define H_PORT2 &PORTC
#define H_PIN1 0
#define H_PIN2 2
#define H_PIN3 4
#define H_PIN4 6
*/



#endif /* INC_ROBOTCONTROL_H_ */
