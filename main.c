#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "system.h" 
#include "i2c.h"
#include "tim3.h"


uint8_t I3G4250D_Read_CTRL_REG1(void);

int main(void)
{
	RCC_Init();
	sysTickInit();
	GPIO_I2C_Init();
	
	
	I2C_init();
	I2C1_ctrl_reg_gyro();
	TIM3_Init_1kHz();
	
	int16_t gx, gy, gz;
	
  while(1)
	{
		if (sensor_ready) 
		{
			sensor_ready = 0;
			
			I3G4250D_ReadGyro(&gx, &gy, &gz);

		}
	}
}

