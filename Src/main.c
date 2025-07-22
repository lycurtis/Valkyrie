#include "stm32f4xx.h"
#include "i2c.h"
#include "mpu6050.h"
#include "complementary.h"
#include "uart.h"
#include "FreeRTOS.h"

comp imu_CA;
volatile uint32_t systick_ticks = 0;

float roll;
float pitch;

char key;

void SysTick_Handler(void){
	systick_ticks++;
}

int main(void){

	SCB->CPACR |= (0xF << 20); // Enable full access to CP10 and CP11 (the FPU)

	I2C_Init();
	MPU6050_Init();
	UART2_Init(115200);

	// Configure SysTick for a 1ms interrupt
	// 16MHz HSI clock
	SysTick_Config(16000000/1000);

	MPU6050_Calibrate();

	ComplementaryFilter_Init(&imu_CA);

	const float loop_dt = 0.01f; // 10ms loop time -> 100Hz frequency
	uint32_t loop_start_tick;

	while(1){
		loop_start_tick = systick_ticks;

		float ax_g, ay_g, az_g;
		float gx_dps, gy_dps, gz_dps;

		MPU6050_Read_Calibrated(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);


		ComplementaryFilter_Update(&imu_CA, ax_g, ay_g, az_g, gx_dps, gy_dps, loop_dt);

		// results
		roll = imu_CA.roll;
		pitch = imu_CA.pitch;

		// Maintain loop frequency
		while((systick_ticks - loop_start_tick) < 10){} //wait 10ms
	}
}




