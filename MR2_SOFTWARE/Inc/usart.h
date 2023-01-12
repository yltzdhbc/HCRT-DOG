/**
  ******************************************************************************
  * File Name          : USART.h

 Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "robocon.h"
/* USER CODE BEGIN Includes */

#define EN_USART_RX 1
#define USART_REC_LEN  			2000 	//�����������ֽ��� 2000
#define RXBUFFERSIZE   1 //�����С
extern uint16_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART_RX_STA;         		//����״̬���
extern uint16_t aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer

#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define USART2TXBUFSIZE   20 //�����С
#define USART2RXBUFSIZE   1 //�����С
extern uint8_t USART2TxBuf[USART2TXBUFSIZE];
extern uint8_t USART2RxBuf[USART2RXBUFSIZE];
extern uint16_t USART2_RX_STA;       //����״̬���

#define EN_USART6_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define USART6TXBUFSIZE   8 //�����С
#define USART6RXBUFSIZE   40 //�����С
extern uint8_t USART6TxBuf[USART6TXBUFSIZE];
extern uint8_t USART6RxBuf[USART6RXBUFSIZE];
extern uint16_t USART6_RX_STA;       //����״̬���

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
