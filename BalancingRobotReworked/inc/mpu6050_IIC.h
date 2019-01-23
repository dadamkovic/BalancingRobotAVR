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

#define MPUADDRESS_READ 0b11010001      //< Read address of MPU6050 for IIC communication.
#define MPUADDRESS_WRITE 0b11010000     //< Write address of MPU6050 for IIC communication.
#define MPUACCEL_REGISTER 0x3B          //< First Register address containing MPU6050 acceleration data
#define RAD_TO_DEG 57.2957
#define DEG_TO_RAD 0.017453
#define PI 3.14159
#define GYRO_CONSTANT 131.0
#define RIGHT_ANGLE_RAD 1.570795
#define BUZZER_PIN 0
#define BUZZER_DDR DDRL
#define BUZZER_PORT PORTL
#define BUZZER_INIT BUZZER_DDR |= _BV(BUZZER_PIN)
#define BUZZER_ON BUZZER_PORT |= _BV(BUZZER_PIN)
#define BUZZER_OFF BUZZER_PORT &= ~_BV(BUZZER_PIN)

uint8_t initIIC();
void IICsendStart();
void IICsendStop();
void IICsendData(uint8_t);
uint8_t IICreadAck();
uint8_t IICreadNack();

uint8_t IICReadMPU(float* dataOut,uint8_t returnRaw);
uint8_t calibrate(float*, uint16_t);

#endif /* MPU6050_IIC_H_ */
