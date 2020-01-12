/**
 * \file communicationFunctions.h
 * \author Daniel Adamkovic
 * \date 20/1/2019
 */

#ifndef COMMUNICATIONFUNCTION_H
#define COMMUNICATIONFUNCTION_H

#include "constants.h"

/**
 * \defgroup COMMUNICATION
 * \brief Includes information about functions that take care of communicating with the robot.
 * \{
 */

/**
 * \brief Sends robot up to three bytes long command that idenitifies the type of action requested by the user.
 * \param[in] command One byte identifier that determines how will the robot treat the information bytes.
 * \param[in] data0 MSB of the data or the entire command(in case the second byte is set to 0x00).
 * \param[in] data1 LSB of the data sent to robot.
 */
void sendRobotCommand(uint8_t command ,uint8_t data0=0,uint8_t data1=0);

/**
 * \brief Will request battery level report fro the robot, if no answer is recieved within a given timeframe returns 30.
 */
uint8_t requestBatteryUpdate();

/**
 * \brief Samples joypad data, processes it and sends it to the robot, for speed and steering control.
 * \param[in] controlSensitivity Determines the behaviour of the filter appplied to samples values. 
 * 
 * \note Higher \p controlSensitivity means weaker filter (0-100).
 */
void handleControl(uint8_t);

/**
 * \brief Requests current angle from robot.
 */

float getRobotAngle();

/**
 * \brief Requests current speed from robot.
 * \return Speed of the robot in the form REAL_SPEED = RETURNED_SPEED/100.0 or -10000 in case nothing is received.
 */
float getRobotSpeed();
 
float getRobotDistance();

/**
 * \brief Initializes calibration of the MPU6050 sensor, robot has to be laying on its back for this.
 * 
 * Request for MPU6050 calibration for the robot is send. Calibration takes some time, during which the angular velocity and offset from true +/- 90 
 * degrees rotation are measured. 
 * At the end of the calibration we will create averages from the measured values and use them as offsets to the values measured
 * when the robot is in operation.
 */



float getRobotCurrent();


 
void initMPUCalibration();

/**
 * \brief Request for angle offset calibration is send to the robot. Calibration takes some time, in which the robot balances 
 * around its 0 point (robot in vertical position idealy 90 degrees to the ground). During this time measured angles are being averaged
 * and at the end this average is considered the new 0 point.
 */
void initAngleCalibration();

/**
 * \brief Sends robot thee request for returning its regulator parameter (P, I or D).
 * 
 * \param[in] para One byte, either letter 'P', 'I' or 'D'
 * \returns Robot regulator parameter in the form, REAL_PARAMETER = RETURNED_PARAMETER / 100.0 or 10 in case connection is not established
 */
int16_t getRobotRegulatorParameter(uint8_t para);

/**
 * \brief Helper function that modifies passed buffer to contain char represenatation of 4 digit passed number.
 * \param[in] number Number to be represented in char form. E. g. 1234 will become '+12.34\0'
 * \param[out] text Empty buffer of the length at least 7, that will be filled with number representation.
 * 
 * This function is <b> NOT </b> supposed to just turn 16 bit signed integers into their char representation, but rather
 * work with the format where instead of representing real numbers using float we store them with two digit precision as
 * whole numbers, by multipling them by 100. This way for example: 19.34 becomes 1934 in this representation and the fuction will
 * corectly turn it back into "+19.34\0".
 * 
 * \note This funtion will not work correctly with \p number containing anything bigger then +/- 9999.
 */
void num2Str(int16_t number,char *text);

#endif
