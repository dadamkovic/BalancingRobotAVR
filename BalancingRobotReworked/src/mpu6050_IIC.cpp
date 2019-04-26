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
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x6B);
    IICsendData(0x01);
    IICsendStop();
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x19);
    IICsendData(7);
    IICsendData(0x00);
    IICsendData(0x00);
    IICsendData(0x00);
    IICsendStop();
    if(IICcheckConnection()){
        //uart_puts("Spojenie nadviazane \n");
        return 0;
    }
    //else uart_puts("ERROR");
    return 1;
  }

uint8_t IICcheckConnection(){
    uint8_t sensor;
    IICsendStart();
    IICsendData(MPUADDRESS_WRITE);
    IICsendData(0x75);
    IICsendStart();
    IICsendData(MPUADDRESS_READ);
    sensor = IICreadNack();
    IICsendStop();
    if(sensor == SENSOR_OK)return 1;
    return 0;
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

MPU::MPU(){
    initIIC();
    //_delay_ms(1000);
    _delay_ms(200);
    IICReadMPU();
    compXAngle = xAccAngle;
    compYAngle = yAccAngle;
    gyroXAngle = compXAngle;
    gyroYAngle = compYAngle;
    //xCal = eeprom_read_float(&xCalAddr);
    //yCal = eeprom_read_float(&yCalAddr);

    /*eeprom_read_block((void*)&xCal, (const void*)&xCalAddr, 4);
    eeprom_read_block((void*)&yCal, (const void*)&yCalAddr, 4);
    eeprom_read_block((void*)&zCal, (const void*)&zCalAddr, 4);*/
};

void MPU::reset(){
    IICReadMPU();
    compXAngle = xAccAngle;
    compYAngle = yAccAngle;
    gyroXAngle = compXAngle;
    gyroYAngle = compYAngle;
}

/**
 * \brief updates the class parameters holding the current X and Y angles
 * \param[in] dt time between updates in seconds
 */
void MPU::updateValues(float dt){
    IICReadMPU();
    gyroXDt = giveGyroAngle(dt, 'X');
    gyroYDt = giveGyroAngle(dt, 'Y');
    gyroXAngle += (gyroXDt);
    gyroYAngle += (gyroYDt);
    if(compX < FIN_COMP){
        compX += COMP_ADDITION;
        compY += COMP_ADDITION;
    }
    compXAngle = (compX * (compXAngle + gyroXDt) + (1-compX) * xAccAngle);   //serves for foward-backward orientation
    compYAngle = (compY * (compYAngle + gyroYDt) + (1-compY) * yAccAngle);      //serves for sideways orientation

}

/**
 * \brief private function that calculates the angle change
 * \param[in] dt time between angle updates
 * \param[in] c switches between calculation of X-axis angle (X) or Y-axis angle (Y)
 * \return 0
 */
float MPU::giveGyroAngle(float dt, char c){
    if(c=='X'){
        return -(xGyAngle + yGyAngle*((sin(compXAngle)*sin(compYAngle))/cos(compYAngle))+ zGyAngle*((cos(compXAngle)*sin(compYAngle))/(cos(compYAngle))))*dt;
    }
    else if(c=='Y')return -((yGyAngle*cos(compXAngle)) - zGyAngle*sin(compXAngle))*dt;
    return 0;
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

uint8_t MPU::IICReadMPU(){
    while(!IICcheckConnection()){
        initIIC();
        uart_puts("Not OK\n");
    }
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


    float yaw  = atan(accY / sqrt(accX * accX + accZ * accZ));
    float pitch = atan2(-accX, accZ);

    //MPU6050 is mounted upside down so this needs to correct the angle
    if(pitch>RIGHT_ANGLE_RAD)pitch = PI - pitch;
    if(pitch<-RIGHT_ANGLE_RAD)pitch = -PI - pitch;

    // ConvertS to deg/s
    float xGyro = (gyroX / GYRO_CONSTANT) * DEG_TO_RAD;
    float yGyro = (gyroY / GYRO_CONSTANT) * DEG_TO_RAD;
    float zGyro = (gyroZ / GYRO_CONSTANT) * DEG_TO_RAD;
    xAccAngle = yaw;
    yAccAngle = pitch;
    if(calibrationInProgress){
        xGyAngle = xGyro;
        yGyAngle = yGyro;
    }
    else{
        xGyAngle = xGyro+xCal;
        yGyAngle = yGyro+yCal;
    }
    zGyAngle = zGyro;

   return 0;
}

/**
 * \brief Starts calibration routine, robot on the back.
 * \param[out] calibratedValues Buffer that will store values for calibration, min 5 float variables.
 * \param[in] samples Number of samples for calibration.
 */

void MPU::calibrate(uint16_t samples = 1000){
    float bufferSum[5];
    float xCalSum, yCalSum = 0;
    for(uint16_t i=0;i<samples;i++){
            IICReadMPU();
            xCalSum += xGyAngle;
            yCalSum += yGyAngle;
            _delay_ms(2);
    };
    xCal = -xCalSum/(float)samples;
    yCal = -yCalSum/(float)samples;
    //eeprom_update_float(&xCalAddr, xCal);
    //eeprom_update_float(&yCalAddr, yCal);
}


