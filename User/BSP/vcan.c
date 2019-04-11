/**
  ******************************************************************************
  * @file			vcan.c
  * @version		V1.0.0
  * @date			
  * @brief   		
						  
  *******************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include "vcan.h"

void vcan_send_byte(uint8_t date);
void vcan_send_wave_form(void);

short  wave_form_data[6] = {0};

void vcan_send_wave_form(void)
{
	uint8_t i;
	
	vcan_send_byte(0x03);
	vcan_send_byte(0xfc);
	for(i = 0;i<6;i++)
	{
	  vcan_send_byte((wave_form_data[i]&0xff)); //现发送低位 然后发送高位
	  vcan_send_byte((wave_form_data[i]>>8));
 
	}
	vcan_send_byte(0xfc);
	vcan_send_byte(0x03);
}	

//山外调试助手发送波形
void vcan_send_byte(uint8_t date)
{
	//HAL_UART_Transmit(&huart6,&date,1,10);
	HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&date, 1);
	//while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
	
}