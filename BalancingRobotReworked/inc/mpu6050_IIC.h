/**
 * \file mpu6050_IIC.h
 * \author Daniel Adamkovic
 */


#ifndef MPU6050_IIC_H_
#define MPU6050_IIC_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#define NO_RAW 0
#define RAW 1
#define MPUADDRESS_READ 0b11010001      //< Read address of MPU6050 for IIC communication.
#define MPUADDRESS_WRITE 0b11010000     //< Write address of MPU6050 for IIC communication.
#define MPUACCEL_REGISTER 0x3B          //< First Register address containing MPU6050 acceleration data
#define RAD_TO_DEG 57.2957
#define DEG_TO_RAD 0.017453
#define PI 3.14159
#define GYRO_CONSTANT 131.0
#define RIGHT_ANGLE_RAD 1.570795
#define SENSOR_OK 'h'



#define ACC_X_ANGLE MPUData[0]
#define ACC_Y_ANGLE MPUData[1]
#define GYRO_X_CHANGE MPUData[2]
#define GYRO_Y_CHANGE MPUData[3]
#define GYRO_Z_CHANGE MPUData[4]
#define FIN_COMP 0.998
#define INIT_COMP 0.985          //tod check later

extern float EEMEM xCalAddr;
extern float EEMEM yCalAddr;
extern float EEMEM zCalAddr;

class MPU{
    public:
        volatile float currentAngle = 0;
        volatile float gyroXAngle = 0;
        volatile float gyroYAngle = 0;
        volatile float gyroXDt, gyroYDt = 0;
        volatile float MPUData[7];
        volatile float compYAngle, compXAngle = 0;
        volatile float xCal = 0, yCal = 0;
        uint8_t calibrationInProgress = 0;
        MPU();
        void updateValues(float);
        void calibrate(uint16_t);
        void reset();
    private:
        volatile float compX = 0.998;
        volatile float compY = 0.998;
        uint8_t IICReadMPU(uint8_t);
        float giveGyroAngle(float dt, char c);
};

uint8_t initIIC();
void IICsendStart();
void IICsendStop();
void IICsendData(uint8_t);
uint8_t IICreadAck();
uint8_t IICreadNack();
uint8_t IICcheckConnection();

#endif /* MPU6050_IIC_H_ */
