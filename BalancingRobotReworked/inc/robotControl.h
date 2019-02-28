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


#define SEND_CONTROL_INFO 0x01
#define REQ_BATTERY_LVL 0x02
#define REQ_TILT_ANGLE 0x03
#define REQ_MPU_CALIBRATION 0x04
#define REQ_ANGLE_CALIBRATION 0x05
#define REQ_SPEED 0x06
#define REQ_REGULATOR_P 0x07
#define REQ_REGULATOR_I 0x08
#define REQ_REGULATOR_D 0x09
#define CHANGE_P 0x0A
#define CHANGE_I 0x0B
#define CHANGE_D 0x0C

#define H_REGISTER1 &DDRC
#define H_REGISTER2 &DDRC
#define H_PORT1 &PORTC
#define H_PORT2 &PORTC
#define H_PIN1 0
#define H_PIN2 2
#define H_PIN3 4
#define H_PIN4 6




#endif /* INC_ROBOTCONTROL_H_ */