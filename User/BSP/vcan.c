/**
  ******************************************************************************
  * @file			vcan.c
  * @version		V1.0.0
  * @date			
  * @brief	开启了三个通道 使用山外串口调试助手打印波形 可以自己添加通道 方便调试
  *******************************************************************************/
  
#include "vcan.h"

uint8_t VcanTxBuff[10]={0};
short  wave_form_data[3] = {0};

void vcan_send_byte(uint8_t date);
void Vcan_Send_Wave_Data(void);


void VcanGC_task(void *pvParameters)
{
	for(;;)
	{
	Vcan_Send_Wave_Data();
		vTaskDelay(50);
	}

}

void Vcan_Send_Wave_Data(void)
{
	VcanTxBuff[0]=0x03;
	VcanTxBuff[1]=0xfc;
	
	VcanTxBuff[2]=(wave_form_data[0]&0xff);
	VcanTxBuff[3]=(wave_form_data[0]>>8);
	VcanTxBuff[4]=(wave_form_data[1]&0xff);
	VcanTxBuff[5]=(wave_form_data[1]>>8);
	VcanTxBuff[6]=(wave_form_data[2]&0xff);
	VcanTxBuff[7]=(wave_form_data[2]>>8);
	
	VcanTxBuff[8]=0xfc;
	VcanTxBuff[9]=0x03;
	
	
	HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&VcanTxBuff, 10);

}	
