#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "system.h" 
//#include "usart.h"
#include "i2c.h"


int main(void)
{
	RCC_Init();
	sysTickInit();
//	GPIO_USART1_Init();
//	USART1_Init();
//	DMA2_USART1_RX_Init();
	GPIO_I2C_Init();

	//usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
	
  while(1)
	{
		
	}
}
