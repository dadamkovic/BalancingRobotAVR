/*
 * mpu6050_IIC.h
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */

#ifndef MPU6050_IIC_H_
#define MPU6050_IIC_H_

#include <avr/io.h>
#include <inttypes.h>
#include <math.h>

#define MPUADDRESS_READ 0b1101001
#define MPUADDRESS_WRITE 0b1101000
#define MPUACCEL_REGISTER 0x3B
#define RAD_TO_DEG 57.2957


void initIIC();
void IICsendStart();
void IICsendStop();
void IICsendData(uint8_t);
uint8_t readAck();
uint8_t readNack();

void IICreadMPU(uint16_t*,uint8_t);


#endif /* MPU6050_IIC_H_ */
