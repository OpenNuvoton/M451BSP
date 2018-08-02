//
// MPU6050 Driver: 3-axis Gyroscope + 3-axis accelerometer + temperature
//
// Interface: I2C
// pin1: Vcc to Vcc (+5V)
// pin2: Gnd to Gnd
// pin3: SCL to I2C0_SCL/GPC4
// pin4: SDA to I2C0_SDA/GPE0
// pin5: XDA -- N.C.
// pin6: XCL -- N.C.
// pin7: AD0 -- N.C.
// pin8: INT -- N.C.

#include <stdio.h>
#include <stdint.h>
#include "M451Series.h"
#include "sys.h"
#include "gpio.h"
#include "i2c.h"
#include "MPU6050_REG.h"

void I2C_SingleWrite(uint8_t index, uint8_t data)
{
    I2C_START(MPU6050_I2C_PORT);
    I2C_WAIT_READY(MPU6050_I2C_PORT);

    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x08);

    I2C_SET_DATA(MPU6050_I2C_PORT, MPU6050_SlaveAddr);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x18);

    I2C_SET_DATA(MPU6050_I2C_PORT, index);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x28);

    I2C_SET_DATA(MPU6050_I2C_PORT, data);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x28);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI | I2C_CTL_STO);
}

uint8_t I2C_SingleRead(uint8_t index)
{
    uint8_t tmp;
    I2C_START(MPU6050_I2C_PORT);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x08);

    I2C_SET_DATA(MPU6050_I2C_PORT, MPU6050_SlaveAddr);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x18);

    I2C_SET_DATA(MPU6050_I2C_PORT, index);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x28);

    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_STA | I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x10);

    I2C_SET_DATA(MPU6050_I2C_PORT, MPU6050_SlaveAddr + 1);
    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x40);

    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(MPU6050_I2C_PORT);
    while(I2C_GET_STATUS(MPU6050_I2C_PORT) != 0x58);

    tmp = I2C_GET_DATA(MPU6050_I2C_PORT);


    I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_CTL_SI | I2C_CTL_STO);

    return tmp;
}

void Init_MPU6050(void)
{
    I2C_SetSlaveAddr(MPU6050_I2C_PORT, 0, MPU6050_SlaveAddr, I2C_GCMODE_DISABLE);
    I2C_SingleWrite(MPU6050_PWR_MGMT_1, 0x00);  // CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0
    I2C_SingleWrite(MPU6050_SMPLRT_DIV, 0x07);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
    I2C_SingleWrite(MPU6050_CONFIG, 0x06);      // set TEMP_OUT_L, DLPF=2 (Fs=1KHz)
    I2C_SingleWrite(MPU6050_GYRO_CONFIG, 0x18); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
    I2C_SingleWrite(MPU6050_ACCEL_CONFIG, 0x01);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
}

uint16_t Read_MPU6050_AccX(void)
{
    uint8_t LoByte, HiByte;

    LoByte = I2C_SingleRead(MPU6050_ACCEL_XOUT_L); // read Accelerometer X_Low  value
    HiByte = I2C_SingleRead(MPU6050_ACCEL_XOUT_H); // read Accelerometer X_High value

    return ((HiByte << 8) | LoByte);
}

uint16_t Read_MPU6050_AccY(void)
{
    uint8_t LoByte, HiByte;
    LoByte = I2C_SingleRead(MPU6050_ACCEL_YOUT_L); // read Accelerometer X_Low  value
    HiByte = I2C_SingleRead(MPU6050_ACCEL_YOUT_H); // read Accelerometer X_High value
    return ((HiByte << 8) + LoByte);
}

uint16_t Read_MPU6050_AccZ(void)
{
    uint8_t LoByte, HiByte;
    LoByte = I2C_SingleRead(MPU6050_ACCEL_ZOUT_L); // read Accelerometer X_Low  value
    HiByte = I2C_SingleRead(MPU6050_ACCEL_ZOUT_H); // read Accelerometer X_High value
    return ((HiByte << 8) + LoByte);
}

uint16_t Read_MPU6050_GyroX(void)
{
    uint8_t LoByte, HiByte;
    LoByte = I2C_SingleRead(MPU6050_GYRO_XOUT_L); // read Accelerometer X_Low  value
    HiByte = I2C_SingleRead(MPU6050_GYRO_XOUT_H); // read Accelerometer X_High value
    return ((HiByte << 8) + LoByte);
}

uint16_t Read_MPU6050_GyroY(void)
{
    uint8_t LoByte, HiByte;
    LoByte = I2C_SingleRead(MPU6050_GYRO_YOUT_L); // read Accelerometer X_Low  value
    HiByte = I2C_SingleRead(MPU6050_GYRO_YOUT_H); // read Accelerometer X_High value
    return ((HiByte << 8) + LoByte);
}

uint16_t Read_MPU6050_GyroZ(void)
{
    uint8_t LoByte, HiByte;
    LoByte = I2C_SingleRead(MPU6050_GYRO_ZOUT_L); // read Accelerometer X_Low  value
    HiByte = I2C_SingleRead(MPU6050_GYRO_ZOUT_H); // read Accelerometer X_High value
    return ((HiByte << 8) + LoByte);
}
