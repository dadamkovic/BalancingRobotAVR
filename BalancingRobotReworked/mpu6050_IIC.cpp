/*
 * mpu6050_IIC.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */

#include "mpu6050_IIC.h"

/*
 * Initializes TWI peripheral (400kHz)
 */
void initIIC(){
    TWBR = 8;              //(F_CPU)/(16+2*TWBR) = 400kHZ
    TWCR |= _BV(TWEN);
}

/*
 * Holds the program until TWI does what it needs to do
 */

void IICwaitForComplete(){
    loop_until_bit_is_set(TWCR,TWINT);
}

/*
 * Send START bit - SCL line held high, SDA brought low
 */
void IICsendStart(){
    TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();
}

/*
 * Send STOP - SCL line held high, SDA brought high
 */

void IICsendStop(){
    TWCR = _BV(TWSTO) | _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();
}

/*
 * Sends 8 bit data
 */
void IICsendData(uint8_t data){
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();
}

/*
 * Reads data from peripheral and acknowledges its arrival
 */

uint8_t IICreadAck(){
    TWCR = _BV(TWEA) | _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();
    return TWDR;
}

/*
 * Reads data from peripheral, doesn't acknowledges its arrival
 */

uint8_t IICreadNack(){
    TWCR = _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();
    return TWDR;
}

/*
 * Starts communication with MPU6050, takes in 7 words of data and performs computations
 * to return x,y angles from gyro and accelerometer (if returnRaw = 0) or data from MPU (returnRaw = 1)
 */

uint16_t* IICreadMPU(uint16_t* dataOut,uint8_t returnRaw){
    IICsendStart();
    IICsendData(MPUADDRESS_READ);
    IICsendData(MPUACCEL_REGISTER);
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);

    uint16_t data[7];
    for(uint8_t i=0;i<6;i++){
    data[i] = (uint16_t)IICreadAck()<<8;
    data[i] |= IICreadAck();
    }
    data[7] = (uint16_t)IICreadAck()<<8;
    data[7] |= IICreadNack();

    IICsendStop();

    float xAngle  = atan(data[1] / sqrt(data[0] * data[0] + data[2] * data[2])) * RAD_TO_DEG;
    float yAngle = atan2(-data[0], data[2]) * RAD_TO_DEG;
    double xGyro = data[4] / 131.0; // Convert to deg/s
    double yGyro = data[5] / 131.0;

    if(returnRaw){
        for(uint8_t i;i<7;i++){
            dataOut[i] = data[i];
        }
        return dataOut;
    }
    else{
        dataOut[0] = xAngle;
        dataOut[1] = yAngle;
        dataOut[2] = xGyro;
        dataOut[3] = yGyro;    }

    return dataOut;
}
