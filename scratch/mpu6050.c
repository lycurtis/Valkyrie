/*
 * mpu6050.c
 *
 *  Created on: Jun 25, 2025
 *      Author: cly
 */


#include "mpu6050.h"
#include "i2c.h"


//void MPU6050_Init(void){
//	uint8_t data = 0x00; // Wake up device
//	I2C_Mem_Write(MPU6050_I2C_ADDR, MPUR_PWR_MGMT_1, &data, 1);
//}

uint8_t MPU6050_Init(void){
	uint8_t check;
	uint8_t data;

	//Initialize the I2C peripheral
	//I2C_Init(); //Optional placement here or place in main file

	//Check the WHO_AM_I register to confirm communication with the MPU6050
	//The MPU6050's I2C address is 0x68, therefore this register should also contain 0x68
	I2C_Mem_Read(MPU6050_I2C_ADDR, MPU6050_WHO_AM_I, &check, 1);

	if(check == MPU6050_I2C_ADDR){
		//Must wake up device by writing 0 to the Power Management 1 Register
		data = 0x00;
		I2C_Mem_Write(MPU6050_I2C_ADDR, MPUR_PWR_MGMT_1, &data, 1);

		// Set the sample rate. 1kHz/(1+SMPLRT_DIV)
		// Set SMPLRT_DIV to 7 for a 125Hz sample rate
		data = 0x07;
		I2C_Mem_Write(MPU6050_I2C_ADDR, MPUR_SMPLRT_DIV, &data, 1);

		// Configure the accelerometer to a full-scale range of +/- 2g
		data = 0x00;
		I2C_Mem_Write(MPU6050_I2C_ADDR, MPUR_ACCEL_CONFIG, &data, 1);

		// Configure the gyroscope to a full-scale range of +/- 250 degrees/sec
		data = 0x00;
		I2C_Mem_Write(MPU6050_I2C_ADDR, MPUR_GYRO_CONFIG, &data, 1);

		return 1; // success
	}

	return 0; // fail
}

void MPU6050_Read_Accel(int16_t *accel_x, int16_t * accel_y, int16_t *accel_z){
	uint8_t buffer[6]; //stores raw data

	// Read 6 bytes of data starting from the ACCEL_XOUT_H register
	// Note: The MPU will automatically increment the register address after each read
	I2C_Mem_Read(MPU6050_I2C_ADDR, MPUR_ACCEL_XOUT_H, buffer, 6);

	// data is transmitted in big-endian format (MSB first)
	// we combine the high and low bytes for each axis to form a 16-bit signed integer
	*accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
	*accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
	*accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
}

void MPU6050_Read_Gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z){
	uint8_t buffer[6];

	I2C_Mem_Read(MPU6050_I2C_ADDR, MPUR_GYRO_XOUT_H, buffer, 6);

	*gyro_x = (int16_t)(buffer[0] << 8 | buffer[1]);
	*gyro_y = (int16_t)(buffer[2] << 8 | buffer[3]);
	*gyro_z = (int16_t)(buffer[4] << 8 | buffer[5]);
}

//int16_t MPU6050_Read_Temp(void){
//	uint8_t buffer[2]; //raw data
//	int16_t temp;
//
//	// Read 2 bytes of data starting from the TEMP_OUT_H register.
//	I2C_Mem_Read(MPU6050_I2C_ADDR, MPUR_TEMP_OUT_H, buffer, 2);
//
//	// Combine the high and low bytes to form the 16-bit temperature value.
//	temp = (int16_t)(buffer[0] << 8 | buffer[1]);
//
//	// Note: To convert this to degrees Celsius, use the formula from the datasheet:
//	// Temperature in degrees C = (TEMP_OUT / 340.0) + 36.53
//	// conversion will be done in main application logic, not in the driver.
//	return temp;
//}

