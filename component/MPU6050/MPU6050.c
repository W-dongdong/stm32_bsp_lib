#include "MPU6050.h"

#define MPU6050_ADDRESS			0xD0

uint8_t MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	if (I2C_MemWrite(&hi2c2, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10))
	{
		return 1;
	}
	return 0;
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	if (I2C_MemRead(&hi2c2, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 10))
    {
        return Data;
    }
	return 0;
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); //解除睡眠，选择陀螺仪时钟
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00); //六个轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09); //采样分分频为10
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06); //滤波参数给最大
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); //陀螺仪最大量程
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);  //加速度最大量程
}


void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *Temp,
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t RawData[14];
	if(I2C_MemRead(&hi2c2, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, RawData, 14, 10))
	{
		*AccX = (RawData[0] << 8) | RawData[1];
		*AccY = (RawData[2] << 8) | RawData[3];
		*AccZ = (RawData[4] << 8) | RawData[5];
		*Temp = (RawData[6] << 8) | RawData[7];
		*GyroX = (RawData[8] << 8) | RawData[9];
		*GyroY = (RawData[10] << 8) | RawData[11];
		*GyroZ = (RawData[12] << 8) | RawData[13];
	}
}
