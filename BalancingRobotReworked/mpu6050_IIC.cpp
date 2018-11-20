///*
// * mpu6050_IIC.cpp
// *
// *  Created on: Aug 23, 2018
// *      Author: daniel
// */
//

#include "uart.h"
#include "mpu6050_IIC.h"
#include <util/delay.h>

/*
 * Initializes TWI peripheral (400kHz)
 */

void initIIC(){
    TWBR = 3;              //(F_CPU)/(16+2*TWBR*4^TWPS) = 400kHZ
    TWSR &= ~(_BV(TWPS1)|_BV(TWPS0));   //TWPS = 1
    TWCR |= _BV(TWEN);      //enables IIC
    uint8_t sensor;
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x6B);
    IICsendData(0x01);
    IICsendStop();
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x75);
    IICsendStart();
    IICsendData(MPUADDRESS_READ);
    sensor = IICreadNack();
    IICsendStop();
    //    i2cWrite(0x19, i2cData, 4, false)
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x19);
    IICsendData(7);
    IICsendData(0x00);
    IICsendData(0x00);
    IICsendData(0x00);
    IICsendStop();
    if(sensor == 'h')interfaceSendString("Spojenie nadviazane \n");
    else interfaceSendString("ERROR");
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
    //IICwaitForComplete();
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
    TWCR = (_BV(TWEA) | _BV(TWINT) | _BV(TWEN));
    IICwaitForComplete();
    return TWDR;
}

/*
 * Reads data from peripheral, doesn't acknowledges its arrival
 */

uint8_t IICreadNack(){
    TWCR = (_BV(TWINT) | _BV(TWEN));
    IICwaitForComplete();
    return TWDR;
}

/*
 * Starts communication with MPU6050, takes in 7 words of data and performs computations
 * to return x,y angles from gyro and accelerometer (if returnRaw = 0) or data from MPU (returnRaw = 1)
 */

uint8_t IICReadMPU(float* dataOut,uint8_t returnRaw){
    float accX,accY,accZ,tempRaw,gyroX,gyroY,gyroZ;

    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(MPUACCEL_REGISTER);
    IICsendStart();
    IICsendData(MPUADDRESS_READ);
    uint8_t received[14];

    for(uint8_t i=0;i<13;i++)received[i] = IICreadAck();
    received[13] = IICreadNack();
    IICsendStop();

    accX = (int16_t)((received[0] << 8) | received[1]);
    accY = (int16_t)((received[2] << 8) | received[3]);
    accZ = (int16_t)((received[4] << 8) | received[5]);
    tempRaw = (int16_t)((received[6] << 8) | received[7]);
    gyroX = (int16_t)((received[8] << 8) | received[9]);
    gyroY = (int16_t)((received[10] << 8) | received[11]);
    gyroZ = (int16_t)((received[12] << 8) | received[13]);

    /*int16_t xAngle  = atan(data[1] / sqrt(data[0] * data[0] + data[2] * data[2])) * RAD_TO_DEG;
    int16_t yAngle = atan2(-data[0], data[2]) * RAD_TO_DEG;*/

    if(returnRaw){
        dataOut[0] = (float)accX;
        dataOut[1] = (float)accY;
        dataOut[2] = (float)accZ;
        dataOut[3] = (float)tempRaw;
        dataOut[4] = (float)gyroX;
        dataOut[5] = (float)gyroY;
        dataOut[6] = (float)gyroZ;
        return 1;
    }
    else{
        float xAngle  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
        float yAngle = atan2(-accX, accZ) * RAD_TO_DEG;
        if(yAngle>90)yAngle = 180 - yAngle;    //MPU6050 is mounted upside down so this needs to correct the angle
        if(yAngle<-90)yAngle = -180 - yAngle;
        float xGyro = gyroX / 131.0; // Convert to deg/s
        float yGyro = gyroY / 131.0;
        float zGyro = gyroZ / 131.0;
        //interfaceSendFloat(gyroY);
        dataOut[0] = xAngle;
        dataOut[1] = yAngle;
        dataOut[2] = xGyro;
        dataOut[3] = yGyro;
        dataOut[4] = zGyro;

        }

   return 0;
}

/*
 *takes in pointer to a vector where I store results and number of samples for calibration (500 - 1000 would be reasonable)
 *stores the results of average output and returns 0
 */

uint8_t calibrate(float* calibratedValues, float samples){
    float bufferAll[5];
    long bufferSum[5];
    for(uint16_t i=0;i<samples;i++){
            IICReadMPU(bufferAll,0);
            for(uint8_t j=0;j<5;j++)bufferSum[j]+=bufferAll[j];
            _delay_ms(2);
    }
    for(uint8_t i=0;i<5;i++)calibratedValues[i] = bufferSum[i]/samples;

    return 0;
}


