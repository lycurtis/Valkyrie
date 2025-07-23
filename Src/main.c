#include "stm32f4xx.h"
#include "i2c.h"
#include "mpu6050.h"
#include "complementary.h"
#include "uart.h"
#include "FreeRTOS.h"
#include "task.h"

uint32_t SystemCoreClock = 16000000;

// Task handles
TaskHandle_t IMU_TaskHandle = NULL;

// global data
comp imu_CA;
float roll;
float pitch;


void IMU_Task(void *pvParameters){
	const float loop_dt = 0.01f; // 10ms loop time --> 100Hz frequency
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t frequency = pdMS_TO_TICKS(10); // 10ms converted to equivalent number of ticks

	while(1){
		float ax_g, ay_g, az_g;
		float gx_dps, gy_dps, gz_dps;

		MPU6050_Read_Calibrated(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);
		ComplementaryFilter_Update(&imu_CA, ax_g, ay_g, az_g, gx_dps, gy_dps, loop_dt);

		// results
		roll = imu_CA.roll;
		pitch = imu_CA.pitch;

		// Optional print statements here for roll pitch
		char buffer[64];
		snprintf(buffer, sizeof(buffer), "Roll: %.2f\tPitch: %.2f\r\n", roll, pitch);
		UART2_WriteString(buffer);

		vTaskDelayUntil(&lastWakeTime, frequency);
	}
}

int main(void){
	// Enable FPU (Floating point unit)
	SCB->CPACR |= (0xF << 20); // Enable full access to CP10 and CP11 (the FPU)

	I2C_Init();
	MPU6050_Init();
	UART2_Init(115200);

	MPU6050_Calibrate();
	ComplementaryFilter_Init(&imu_CA);

	xTaskCreate(
			IMU_Task,
			"IMU_Task",
			256,
			NULL,
			tskIDLE_PRIORITY + 1,
			&IMU_TaskHandle
			);

	// Start RTOS scheduler
	vTaskStartScheduler();


	while(1){}
}




