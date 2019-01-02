#ifndef COMMUNICATIONFUNCTION_H
#define COMMUNICATIONFUNCTION_H

#include "constants.h"

void sendRobotCommand(uint8_t,uint8_t data0=0,uint8_t data1=0);
uint8_t requestBatteryUpdate();
void handleControl(uint8_t);
float getRobotAngle();
int16_t getRobotSpeed();
void initMPUCalibration();
void initAngleCalibration();
int16_t getRobotRegulatorParameter(uint8_t);
void num2Str(int16_t,char*);

#endif
