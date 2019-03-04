/**
 * \file mpu6050_IIC.cpp
 * \author Daniel Adamkovic
 * \brief Includes declarations of all functions that interact with MPU6050
 */

#include "uart.h"
#include "mpu6050_IIC.h"
#include <util/delay.h>

/**
 * \brief Called <b> once </b> to initialize the MPU6050.
 * \return 0 if initialization succeeds, 1 otherwise.
 *
 * The function sets up the IIC peripheral to 400kHz clock
 * and then sends data to MPU60 which enable its functionality. Test of correct
 * connection is carried out.
 * Initializes TWI peripheral (400kHz)
 */

uint8_t initIIC(){
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
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x19);
    IICsendData(7);
    IICsendData(0x00);
    IICsendData(0x00);
    IICsendData(0x00);
    IICsendStop();
    if(sensor == 'h'){
        //uart_puts("Spojenie nadviazane \n");
        return 0;
    }
    //else uart_puts("ERROR");
    return 1;
  }


/**
 * \brief Holds the prorgram execution until IIC is done.
 */

void IICwaitForComplete(){
    loop_until_bit_is_set(TWCR,TWINT);
}

/**
 * \brief Sends START bit - SCL line held high, SDA brought low.
 */
void IICsendStart(){
    TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();

}

/*
 * \brief Sends STOP bit - SCL line held high, SDA brought high
 * \return void
 */

void IICsendStop(){
    TWCR = _BV(TWSTO) | _BV(TWINT) | _BV(TWEN);
    //IICwaitForComplete();
}

/**
 * \brief Sends 8 bits of data through IIC.
 * \return void
 */
void IICsendData(uint8_t data){
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    IICwaitForComplete();
}

/**
 * \brief Reads 8 bits of data from peripheral and acknowledges its arrival.
 * \return Byte of read data.
 */

uint8_t IICreadAck(){
    TWCR = (_BV(TWEA) | _BV(TWINT) | _BV(TWEN));
    IICwaitForComplete();
    return TWDR;
}

/**
 * \brief Reads 8 bits of data from peripheral no ACK.
 * \see IICreadAck
 * \return Byte of read data.
 */

uint8_t IICreadNack(){
    TWCR = (_BV(TWINT) | _BV(TWEN));
    IICwaitForComplete();
    return TWDR;
}
/**
 * \brief COnsructor of the MPU class, performs initialization of mpu6050 and loads initial values
 */

MPU::MPU(){
    initIIC();
    _delay_ms(1000);
    IICReadMPU(NO_RAW);
    compXAngle = MPUData[0];
    compYAngle = MPUData[1];
    gyroXAngle = compXAngle;
    gyroYAngle = compYAngle;
};

/**
 * \brief Loads new values from mpu6050 and updates internal parameters accordingly.
 */
void MPU::updateValues(float dt){
    IICReadMPU(NO_RAW);
    gyroXDt = giveGyroAngle(dt, 'X');
    gyroYDt = giveGyroAngle(dt, 'Y');
    gyroXAngle += (gyroXDt);
    gyroYAngle += (gyroYDt);

    compXAngle = (0.995 * (compXAngle + gyroXDt) + 0.005 * ACC_X_ANGLE);   //serves for foward-backward orientation
    compYAngle = (0.99 * (compYAngle + gyroYDt) + 0.01 * ACC_Y_ANGLE);      //serves for sideways orientation

}

/**
 * \brief Private method of the MPU class, used to calculate angle change from gyroscope readings.
 * \param[in] dt Time between individual samples (i.e. readings from mpu).
 * \param[in] c Either 'X' or 'Y' depending on which axis we calculate angle change for.
 * \param[out] 0 Calculated value, or -1000 in case of error.
 */
float MPU::giveGyroAngle(float dt, char c){
    if(c=='X'){
        return -(GYRO_X_CHANGE + GYRO_Y_CHANGE*((sin(compXAngle)*sin(compYAngle))/cos(compYAngle))+ GYRO_Z_CHANGE*((cos(compXAngle)*sin(compYAngle))/(cos(compYAngle))))*dt;
    }
    else if(c=='Y')return -((GYRO_Y_CHANGE*cos(compXAngle)) - GYRO_Z_CHANGE*sin(compXAngle))*dt;
    return -1000;
}
/**
 * \brief Queries MPU6050 for data and processes it.
 * \param[in] returnRaw If set to 'R' or 'r', function returns data from MPU6050, without processing it.
 * \param[out] dataOut Address of buffer containing at least 5 float variables (7 in case return raw is 'r' or 'R').
 * \return Returns 1 in case returnRaw is set, 0 otherwise.
 *
 * Starts communication with MPU6050. Reads 14 bytes of data and sets it in the buffer (returnRaw set)
 * or performs computations and sets yaw, pitch from accelerometer and angle accelerations from gyroscope.
 */

uint8_t MPU::IICReadMPU(uint8_t returnRaw){
    float accX,accY,accZ,tempRaw,gyroX,gyroY,gyroZ;
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(MPUACCEL_REGISTER);
    IICsendStart();
    IICsendData(MPUADDRESS_READ);
    uint8_t received[14];

    //Reading 14 bytes of data.
    for(uint8_t i=0;i<13;i++)received[i] = IICreadAck();
    received[13] = IICreadNack();
    IICsendStop();

    //Shifting the read data.
    accX = (int16_t)((received[0] << 8) | received[1]);
    accY = (int16_t)((received[2] << 8) | received[3]);
    accZ = (int16_t)((received[4] << 8) | received[5]);
    tempRaw = (int16_t)((received[6] << 8) | received[7]);
    gyroX = (int16_t)((received[8] << 8) | received[9]);
    gyroY = (int16_t)((received[10] << 8) | received[11]);
    gyroZ = (int16_t)((received[12] << 8) | received[13]);


    if(returnRaw){
        MPUData[0] = (float)accX;
        MPUData[1] = (float)accY;
        MPUData[2] = (float)accZ;
        MPUData[3] = (float)tempRaw;
        MPUData[4] = (float)gyroX;
        MPUData[5] = (float)gyroY;
        MPUData[6] = (float)gyroZ;
        return 1;
    }
    else{
        float xAngle  = atan(accY / sqrt(accX * accX + accZ * accZ));
        float yAngle = atan2(-accX, accZ);
        if(yAngle>RIGHT_ANGLE_RAD)yAngle = PI - yAngle;    //MPU6050 is mounted upside down so this needs to correct the angle
        if(yAngle<-RIGHT_ANGLE_RAD)yAngle = -PI - yAngle;
        //BUZZER_ON;
        // ConvertS to deg/s
        float xGyro = (gyroX / GYRO_CONSTANT) * DEG_TO_RAD;
        float yGyro = (gyroY / GYRO_CONSTANT) * DEG_TO_RAD;
        float zGyro = (gyroZ / GYRO_CONSTANT) * DEG_TO_RAD;
        float tmp = gyroXOffset;
        MPUData[0] = xAngle;
        MPUData[1] = yAngle;
        MPUData[2] = xGyro - tmp;
        tmp = gyroYOffset;
        MPUData[3] = yGyro - tmp;
        tmp = gyroZOffset;
        MPUData[4] = zGyro - tmp;
        }
   return 0;
}

/**
 * \brief Starts calibration routine, robot laying on the back.
 *
 * Collects a given amount of gyroscope samples from MPU and calculates their mean. If the robto is still it should be 0 in case the
 * measurement is perfect, in reality it will be a small value representing the gyroscope acceleration measure. We can later use this
 * calculated mean to correct every gyroscope reading.
 *
 * \note CAlling this method breaks the flow of the program and halts its execution for 4+ seconds. User is warned about this by a sharp
 * beeping sound at the beginning, middle and end of the calibration.
 */

void MPU::initGyroCalibration(){
    BUZZER_ON;
    _delay_ms(2000);
    OCR1A = 0;
    OCR1B = 0;
    BUZZER_OFF;
    float xErr = 0;
    float yErr = 0;
    float zErr = 0;
    for(uint16_t i=0;i<1000;i++){
        //if(i%2 == 0)BUZZER_TOGGLE;
        IICReadMPU(NO_RAW);
        xErr += GYRO_X_CHANGE;
        yErr += GYRO_Y_CHANGE;
        zErr += GYRO_Z_CHANGE;
        _delay_ms(2);
    }
    gyroXOffset = xErr/1000.0;
    gyroYOffset = xErr/1000.0;
    gyroZOffset = xErr/1000.0;
    uart_putf(gyroXOffset);
    uart_putc('\n');
    BUZZER_ON;
    _delay_ms(2000);
    BUZZER_OFF;
}


