#include "stm32f4xx.h"
#include "mpu6050.h"

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

float accel_x_g, accel_y_g, accel_z_g;

int main(void){

	SCB->CPACR |= (0xF << 20); // Enable full access to CP10 and CP11 (the FPU)

	I2C_Init();
	MPU6050_Init();


	while(1){
		MPU6050_Read_Accel(&accel_x, &accel_y, &accel_z);
		MPU6050_Read_Gyro(&gyro_x, &gyro_y, &gyro_z);

		accel_x_g = accel_x / 16384.0f;
		accel_y_g = accel_y / 16384.0f;
		accel_z_g = accel_z / 16384.0f;
	}
}
