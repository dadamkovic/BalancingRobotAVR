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
#define REQ_DISTANCE 0x08
#define REQ_CURRENT 0x09
#define DATA_TRANSFER 0xf0





#endif /* INC_ROBOTCONTROL_H_ */
