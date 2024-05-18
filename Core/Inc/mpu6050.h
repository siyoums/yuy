/*
 * mpu6050.h
 *
 *  Created on: May 3, 2024
 *      Author: siyoums
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "stm32f4xx_hal.h"


// MPU6050 structure
typedef struct
{

    uint8_t Rec_Data[6];
	float temprature_raw;
    float Accel_X_RAW;
    float Accel_Y_RAW;
    float Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;


    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    double KalmanAngleX;
    double KalmanAngleY;

} MPU6050_t;


uint8_t mpu6050_init(I2C_HandleTypeDef *I2Cx, MPU6050_t *imu);
uint8_t mpu6050_read_accel_dma(I2C_HandleTypeDef* i2c_handle, MPU6050_t *imu);
void mpu6050_read_accel_dma_complete(I2C_HandleTypeDef *i2c_handle, MPU6050_t *imu);
void mpu6050_process_after_filter(I2C_HandleTypeDef * i2c_handle, MPU6050_t *imu);

#endif /* INC_MPU6050_H_ */
