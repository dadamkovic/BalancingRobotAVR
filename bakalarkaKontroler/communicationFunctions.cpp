#include <SoftwareSerial.h>
#include <avr/io.h>
#include "constants.h"
#include <arduino.h>

void sendRobotCommand(uint8_t command,uint8_t data0=0,uint8_t data1=0){
  Serial.print(command);
  if(data0){
    Serial.print(data0);
  }
  if(data1){
    Serial.print(data1);
  }
}
  


uint8_t requestBatteryUpdate(){
  sendRobotCommand(REQ_BATTERY_LVL);
  unsigned long waitTime = millis();
  uint8_t charge;
  while(!Serial.available()){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return 30;
  }
  return Serial.read();
}

/*
 * INPUT: control sensitivity - value between 0 - 100, fed into complementary filter
 *                              hiegher means more sensitive
 */

void handleControl(uint8_t controlSensitivity){
  uint8_t joypadXPos = map(analogRead(JOYPAD_X),0,255,0,100);
  uint8_t joypadYPos = map(analogRead(JOYPAD_Y),0,255,0,100);
  static uint8_t joypadXPosOld, joypadYPosOld = 0;
  joypadXPosOld = joypadXPos;
  joypadYPosOld = joypadYPos;
  joypadXPos = uint8_t((joypadXPosOld*(100 - controlSensitivity) + joypadXPos*controlSensitivity)/100.0);
  joypadYPos = uint8_t((joypadYPosOld*(100 - controlSensitivity) + joypadYPos*controlSensitivity)/100.0);
  sendRobotCommand(SEND_CONTROL_INFO,joypadXPos,joypadYPos);
};


float getRobotAngle(){
  unsigned long waitTime = millis();
  sendRobotCommand(REQ_TILT_ANGLE);
  int16_t angle;
  while(!Serial.available()){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return -10000;
  }
  angle = Serial.read()<<8;
  angle |= Serial.read();
  return float(angle/100.0);
}

void initMPUCalibration(){
  sendRobotCommand(REQ_MPU_CALIBRATION);
}

void initAngleCalibration(){
  sendRobotCommand(REQ_ANGLE_CALIBRATION);
}

int16_t getRobotSpeed(){
  unsigned long waitTime = millis();
  sendRobotCommand(REQ_SPEED);
  int16_t curSpeed;
  while(!Serial.available()){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return -10000;
  }
  curSpeed = Serial.read()<<8;
  curSpeed |= Serial.read();
  return curSpeed;
}

int16_t getRobotRegulatorParameter(uint8_t para){
  unsigned long waitTime = millis();
  if(para == 'P')sendRobotCommand(REQ_REGULATOR_P);
  else if(para == 'I')sendRobotCommand(REQ_REGULATOR_P);
  else if(para == 'D')sendRobotCommand(REQ_REGULATOR_D);
  int16_t parameter;
  while(!Serial.available()){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return 10;
  }
  parameter = Serial.read()<<8;
  parameter |= Serial.read();
  return parameter;
}





/*
 * INPUT: is in the form of 4 digit number, plus sign
 * e.g. 2000 == 20.00(real),202 == 2.02(real)
 */

void num2Str(int16_t number,char* text){
    if(number<0){
    text[0] = '-'; 
    number*=-1;
  }
  else{
    text[0] = '+';
  }
  
  text[1] = '0' + number / 1000;
  text[2] = '0' + (number % 1000)/100;
  text[3] = '.';
  text[4] = '0' + (number % 100)/10;
  text[5] = '0' + (number % 10);
  text[6] = '\0';
}



