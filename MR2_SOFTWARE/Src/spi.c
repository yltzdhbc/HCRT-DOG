/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi5;

/* SPI5 init function */
void MX_SPI5_Init(void)
{

    hspi5.Instance = SPI5;
    hspi5.Init.Mode = SPI_MODE_MASTER;
    hspi5.Init.Direction = SPI_DIRECTION_2LINES;
    hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi5.Init.NSS = SPI_NSS_SOFT;
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi5.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        Error_Handler();
    }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(spiHandle->Instance==SPI5)
    {
        /* USER CODE BEGIN SPI5_MspInit 0 */

        /* USER CODE END SPI5_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI5_CLK_ENABLE();

        /**SPI5 GPIO Configuration
        PF7     ------> SPI5_SCK
        PF9     ------> SPI5_MOSI
        PF8     ------> SPI5_MISO
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* USER CODE BEGIN SPI5_MspInit 1 */

        /* USER CODE END SPI5_MspInit 1 */
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

    if(spiHandle->Instance==SPI5)
    {
        /* USER CODE BEGIN SPI5_MspDeInit 0 */

        /* USER CODE END SPI5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI5_CLK_DISABLE();

        /**SPI5 GPIO Configuration
        PF7     ------> SPI5_SCK
        PF9     ------> SPI5_MOSI
        PF8     ------> SPI5_MISO
        */
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_8);

    }
    /* USER CODE BEGIN SPI5_MspDeInit 1 */

    /* USER CODE END SPI5_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
