/*
 * mpu6050.c
 *
 *  Created on: May 3, 2024
 *      Author: siyoums
 */

#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define MPU6050_CONFIG 0X1A
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_INT_PIN_CFG 0X37
#define MPU6050_INT_ENABLE 0X38
#define MPU6050_INT_STATUS 0X3A

// Setup MPU6050
#define MPU6050_ADDR 0xD0 //

const uint16_t i2c_timeout = 100;
const double  Accel_Z_corrector = 14418.0;

uint8_t mpu6050_init(I2C_HandleTypeDef *I2Cx, MPU6050_t* imu) {
    uint8_t check;
	uint8_t Data;

    HAL_Delay(50);

    // check device ID WHO_AM_ I
  HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

	   if (check == 104) // 0x68 will be returned by the sensor if everything goes well
	     {
	         // reset, clksel, power management register 0X6B we should write all 0's to wake the sensor up
	         Data = 0x80;
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
	         HAL_Delay(100);

	         // select clock source
	         Data = 0x09;
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

	         // wake up sensor

	         // Set sample RATE of 100Hz by writing SMPLRT_DIV register (not data output rate (1kz for acc)
	         Data = 79;
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);
	         HAL_Delay(50);

	         //dlpf
	         Data = 0;
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_CONFIG, 1, &Data, 1, i2c_timeout);
	         HAL_Delay(50);

	         // Set accelerometer configuration in ACCEL_CONFIG Register
	         // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
	         Data = 0x00;
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

	         // Set Gyroscopic configuration in GYRO_CONFIG Register
	         // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
	         Data = 0x00;
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

	         // Configure INT pin to generate an interrupt whenever data is available / conversion complete
	         uint8_t INT_LEVEL = 0x0; // 0 - active high, 1 - active low
	         uint8_t LATCH_INT_EN = 0x0; // /0 - INT 50us pulse, 1 - interrupt clear required
	         uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation

	         Data = (INT_LEVEL<<7 | LATCH_INT_EN<<5 | INT_RD_CLEAR<<4);
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_INT_PIN_CFG, 1, &Data, 1, i2c_timeout);
	         HAL_Delay(50);

	         //interrupt enable settings
	     	uint8_t DATA_RDY_EN = 0x1; // 1 - enable, 0 - disable
	         HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_INT_ENABLE, 1, &DATA_RDY_EN, 1, i2c_timeout);
	         HAL_Delay(50);


	         return 0;
	     }
	   return 1;


}

uint8_t mpu6050_read_accel_dma(I2C_HandleTypeDef* i2c_handle, MPU6050_t *imu) {
	if (HAL_I2C_Mem_Read_DMA(i2c_handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, imu->Rec_Data, 6) == HAL_OK) {
		return 1;
	}
	return 0;
}

// process imu data
void mpu6050_read_accel_dma_complete(I2C_HandleTypeDef *i2c_handle, MPU6050_t *imu) {

	imu->Accel_X_RAW = (int16_t)(imu->Rec_Data[0] << 8 | imu->Rec_Data[1]);
	imu->Accel_Y_RAW = (int16_t)(imu->Rec_Data[2] << 8 | imu->Rec_Data[3]);
	imu->Accel_Z_RAW = (int16_t)(imu->Rec_Data[4] << 8 | imu->Rec_Data[5]);

//	imu->Ax = imu->Accel_X_RAW / 16384.0;
//	imu->Ay = imu->Accel_Y_RAW / 16384.0;
//	imu->Az = imu->Accel_Z_RAW / Accel_Z_corrector;
}

void mpu6050_process_after_filter(I2C_HandleTypeDef * i2c_handle, MPU6050_t *imu) {
		imu->Ax = imu->Accel_X_RAW / 16384.0;
		imu->Ay = imu->Accel_Y_RAW / 16384.0;
		imu->Az = imu->Accel_Z_RAW / Accel_Z_corrector;
}

//uint8_t mpu6050_read_accel_dma(I2C_HandleTypeDef *I2Cx, MPU6050_t *data_struct){
//	if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data_struct->Rec_Data, 6, i2c_timeout) == HAL_OK) {
//		return 1;
//	}
//	return 0;
//}
//
//void mpu6050_read_accel_complete_dma(MPU6050_t *data_struct){
//	data_struct->Accel_X_RAW = (int16_t)(data_struct->Rec_Data[0] << 8 | data_struct->Rec_Data[1]);
//	data_struct->Accel_Y_RAW = (int16_t)(data_struct->Rec_Data[2] << 8 | data_struct->Rec_Data[3]);
//	data_struct->Accel_Z_RAW = (int16_t)(data_struct->Rec_Data[4] << 8 | data_struct->Rec_Data[5]);
//
//	data_struct->Ax = data_struct -> Accel_X_RAW / 16384.0;
//	data_struct->Ay = data_struct -> Accel_Y_RAW / 16384.0;
//	data_struct->Az = data_struct -> Accel_Z_RAW / Accel_Z_corrector;
//
//
//}

//void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
//{
//    uint8_t Rec_Data[6];
//
//    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
//
//    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
//
//    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
//    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
//    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
//
//    /*** convert the RAW values into acceleration in 'g'
//         we have to divide according to the Full scale value set in FS_SEL
//         I have configured FS_SEL = 0. So I am dividing by 16384.0
//         for more details check ACCEL_CONFIG Register              ****/
//
//    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
//}

