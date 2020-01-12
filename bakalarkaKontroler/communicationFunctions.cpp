/**
 * \file communicationFunctions.cpp
 * \author Daniel Adamkovic
 * \date 20/1/2019
 */

#include <SoftwareSerial.h>
#include <avr/io.h>
#include "constants.h"
//#include <arduino.h>

extern SoftwareSerial BTSerial;

/**
 * \addtogroup COMMUNICATION
 * \{
 */

/*
void sendRobotCommand(uint8_t command,uint8_t data0=0,uint8_t data1=0){
  BTSerial.write(command);                
  //BTSerial.write(data0);      //MSB of the data being sent
  if(data1){                  //LSB of the data being sent, if 0 it's ignored
    BTSerial.write(data1);
  }
}*/
void sendRobotCommand(uint8_t command,uint8_t data0=0,uint8_t data1=0){
  BTSerial.write(command);                
  if(data0)BTSerial.write(data0);      //MSB of the data being sent
  if(data1)BTSerial.write(data1);
}
  
  
uint8_t requestBatteryUpdate(){
  sendRobotCommand(REQ_BATTERY_LVL);
  uint8_t charge;                     //will store the returned chage (0-100)
  //while(BTSerial.available()!=1){};
  return BTSerial.read();             //returns read data from the robot
}


void initMPUCalibration(){
  sendRobotCommand(REQ_MPU_CALIBRATION);
}

void handleControl(uint8_t timeDelay){
  uint8_t joypadXPos = map(constrain(analogRead(JOYPAD_X)-170,0,1000),0,911,1,100);     //maps 1 bit value to something represantable by 8 bits
  uint8_t joypadYPos = map(constrain(analogRead(JOYPAD_Y)-170,0,1000),0,911,1,100);

  //uint8_t command = 0b10000000 | (joypadYPos << 4) | joypadXPos; 

  sendRobotCommand(CONTROL_INFO,joypadXPos, joypadYPos);

  delay(timeDelay);
};


float getRobotAngle(){
  sendRobotCommand(REQ_TILT_ANGLE);
  /*int16_t currAngle;
  uint8_t proceed = 0;
  uint8_t tmp[3];
  int i = 0;
  while(i<10){
    BTSerial.readBytes(tmp,3);
    if(tmp[0] == DATA_TRANSFER)break;
    i++;
  }
  currAngle = tmp[1]<<8;           //MSB
  currAngle |= tmp[2];             //LSB
  return (float(currAngle)/100.0)*RAD_TO_DEG;
  //immediately transforms value into float representation and returns it*/
  while(!BTSerial.available()){};
  float tmp;
  tmp = BTSerial.parseFloat();
  tmp *= RAD_TO_DEG;
  if(tmp>180)tmp=360-tmp;
  return tmp;
}

float getRobotDistance(){
  //BTSerial.flush();
  sendRobotCommand(REQ_DISTANCE);
  /*int16_t distance;
  while(!BTSerial.available()==2){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return -10000;     //returns -10000 if the communication doesn't arrive in time
  }
  distance = BTSerial.read()<<8;           //MSB
  distance |= BTSerial.read();             //LSB*/
  float tmp;
  tmp = BTSerial.parseFloat();
  //if(tmp>1000 || tmp<-1000) return 0;
  return tmp;
  //return distance;            //immediately transforms value into float representation and returns it
}

float getRobotSpeed(){        //see getRobotAngle above
  //BTSerial.flush();
  sendRobotCommand(REQ_SPEED);
  /*int16_t curSpeed;
  while(!BTSerial.available()){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return -10000;
  }
  curSpeed = BTSerial.read()<<8;
  curSpeed |= BTSerial.read();
  return curSpeed;*/
  float tmp;
  tmp = BTSerial.parseFloat();
  //if(tmp>1000 || tmp<-1000) return 0;
  return tmp;
}


float getRobotCurrent(){        //see getRobotAngle above
  //unsigned long waitTime = millis();
  BTSerial.flush();
  sendRobotCommand(REQ_CURRENT);
  /*int16_t curCurrent;
  while(!BTSerial.available()){
    if((millis() - waitTime)>ALLOWED_WAIT_TIME)return -10000;
  }
  curCurrent = BTSerial.read()<<8;
  curCurrent |= BTSerial.read();
  return curCurrent;*/
  float tmp;
  tmp = BTSerial.parseFloat();
  return tmp;
}


/*
 * INPUT: is in the form of 4 digit number, plus sign
 * e.g. 2000 == 20.00(real),202 == 2.02(real)
 */
void num2Str(int16_t number,char* text){
  if(number<0){       //adds appropriate sign
    text[0] = '-'; 
    number*=-1;
  }
  else{
    text[0] = '+';
  }
  
  text[1] = '0' + (number / 1000);
  text[2] = '0' + ((number % 1000)/100);
  text[3] = '.';
  text[4] = '0' + ((number % 100)/10);
  text[5] = '0' + (number % 10);
  text[6] = '\0';
}

/**
 * \}
 */
