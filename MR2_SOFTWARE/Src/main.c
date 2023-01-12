
#include "robocon.h"

/* TASK ------------------------------------------------------------------*/

#define START_TASK_PRIO		0
#define START_STK_SIZE 		128
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define MotorControl_TASK_PRIO		4
#define MotorControl_STK_SIZE 		256
TaskHandle_t MotorControlTask_Handler;
void MotorControl_task(void *pvParameters);

#define PostureControl_TASK_PRIO		5
#define PostureControl_STK_SIZE 		256
TaskHandle_t PostureControlTask_Handler;
void PostureControl_task(void *pvParameters);

#define Navi_TASK_PRIO		5
#define Navi_STK_SIZE 		256
TaskHandle_t NaviTask_Handler;
void Navi_task(void *pvParameters);

#define Detect_TASK_PRIO		6
#define Detect_STK_SIZE 		128
TaskHandle_t DetectTask_Handler;
void Detect_task(void *pvParameters);

#define Debug_TASK_PRIO		6
#define Debug_STK_SIZE 		256
TaskHandle_t DebugTask_Handler;
void Debug_task(void *pvParameters);

#define Rc_TASK_PRIO		6
#define Rc_STK_SIZE 		256
TaskHandle_t RcTask_Handler;
void Rc_task(void *pvParameters);

#define VcanGC_TASK_PRIO		6
#define VcanGC_STK_SIZE 		256
TaskHandle_t VcanGCTask_Handler;
void VcanGC_task(void *pvParameters);

#define Test_TASK_PRIO		6
#define Test_STK_SIZE 		256
TaskHandle_t TestTask_Handler;
void Test_task(void *pvParameters);

#define LogicalFlow_TASK_PRIO		6
#define LogicalFlow_STK_SIZE 		256
TaskHandle_t LogicalFlowTask_Handler;
void LogicalFlow_task(void *pvParameters);


void SystemClock_Config(void);
static void MX_NVIC_Init(void);


int main(void)
{
    HAL_Init();						//Hal���ʼ��
    SystemClock_Config();	//ϵͳʱ�ӳ�ʼ��
    MX_GPIO_Init();				//GPIO��ʼ��


    MX_DMA_Init();				//DMA��ʼ��
    MX_CAN1_Init();				//CAN1�ӿڳ�ʼ��

    MX_CAN2_Init();				//CAN1�ӿڳ�ʼ��
    MX_SPI5_Init();				//spi5��ʼ��

    MX_USART2_UART_Init();
    uart_receive_init(&USART2_HUART);//USART2DMA�����ж�
    MX_USART3_UART_Init();
    uart_receive_init(&USART3_HUART);//USART3DMA�����ж�   ps2���� ����������Ϊ openmv2�Ľ���
    MX_USART6_UART_Init();
    uart_receive_init(&IMU_HUART);//usart6DMA��������������
    MX_UART7_Init();			//uart7DMA����
    uart_receive_init(&OPENMV_HUART);//USART7DMA�����ж�  openmv
    MX_UART8_Init();		//usart8DMA�������ݸ�ɽ����λ��

    MX_NVIC_Init();				//�ж����ȼ���ʼ��
    //��ʱ��ʱ��90M��Ƶϵ��9000-1,��ʱ��3��Ƶ��90M/9000=10K�Զ���װ��10-1,��ʱ������1ms
    //TIM3_Init(10-1,9000-1);//��ʱ��3��ʼ�� ��̬��ʱ��

    my_can_filter_init_recv_all(&hcan1);//����CAN�˲���
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//����CAN1
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);//����CAN2

    pid_param_init();		//����ʹ�õ���PID������ʼ��
    buzzer_init(500-1, 90-1);//��������ʼ��
    led_configuration();	//��ˮ�� ���̵Ƴ�ʼ��
    //f=Tck/(psc+1)*(arr+1) ��ʱ��ʱ��Ϊ90M  50Hz=180MHz/(90*20000)
    Servo_Init(20000-1,90-1);//���PWMͨ����ʼ��

    TIM2_CH3_Cap_Init(0XFFFFFFFF,90-1); //PWM���� ��1MHZ��Ƶ�ʼ���

    Servo1_OPEN;//���1��λ


//    Servo2_DOWN_POS;//���2��λ
//    Servo3_CLOSE;

    power_ctrl_on_all(); //����ȫ����24V���

    printf("\r\n/*************SYSTEM INIT SUCCESS****************/ \r\n");

    //������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������
    vTaskStartScheduler();          //�����������

}

//��ʼ����������
void start_task(void *pvParameters)
{
    BeginWarnBuzzer();		//������ʾ��

    robomoudle_init();		//��ʼ��robomoudle

    taskENTER_CRITICAL();           //�����ٽ���
    //����MotorControl_task
//    xTaskCreate((TaskFunction_t )MotorControl_task,
//                (const char*    )"MotorControl_task",
//                (uint16_t       )MotorControl_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )MotorControl_TASK_PRIO,
//                (TaskHandle_t*  )&MotorControlTask_Handler);
//    //����PostureControl_task
//    xTaskCreate((TaskFunction_t )PostureControl_task,
//                (const char*    )"PostureControl_task",
//                (uint16_t       )PostureControl_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )PostureControl_TASK_PRIO,
//                (TaskHandle_t*  )&PostureControlTask_Handler);
    //����NAVIGATION����
//    xTaskCreate((TaskFunction_t )Navi_task,
//                (const char*    )"Navi_task",
//                (uint16_t       )Navi_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Navi_TASK_PRIO,
//                (TaskHandle_t*  )&NaviTask_Handler);
    //����Detect����
//    xTaskCreate((TaskFunction_t )Detect_task,
//                (const char*    )"Detect_task",
//                (uint16_t       )Detect_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Detect_TASK_PRIO,
//                (TaskHandle_t*  )&DetectTask_Handler);
    //����Debug_task
//    xTaskCreate((TaskFunction_t )Debug_task,
//                (const char*    )"Debug_task",
//                (uint16_t       )Debug_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Debug_TASK_PRIO,
//                (TaskHandle_t*  )&DebugTask_Handler);
    //����Rc_task
//    xTaskCreate((TaskFunction_t )Rc_task,
//                (const char*    )"Rc_task",
//                (uint16_t       )Rc_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )Rc_TASK_PRIO,
//                (TaskHandle_t*  )&RcTask_Handler);
//    //����VcanGC���� VCAN ground control ɽ����λ��
//    xTaskCreate((TaskFunction_t )VcanGC_task,
//                (const char*    )"VcanGC_task",
//                (uint16_t       )VcanGC_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )VcanGC_TASK_PRIO,
//                (TaskHandle_t*  )&VcanGCTask_Handler);
    //����Test����
    xTaskCreate((TaskFunction_t )Test_task,
                (const char*    )"Test_task",
                (uint16_t       )Test_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Test_TASK_PRIO,
                (TaskHandle_t*  )&TestTask_Handler);
//    //����LogicalFlow���� �߼�������
//    xTaskCreate((TaskFunction_t )LogicalFlow_task,
//                (const char*    )"LogicalFlow_task",
//                (uint16_t       )LogicalFlow_STK_SIZE,
//                (void*           )NULL,
//                (UBaseType_t    )LogicalFlow_TASK_PRIO,
//                (TaskHandle_t*  )&LogicalFlowTask_Handler);

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���

}

void ResetStart(void);

void Test_task(void *pvParameters)
{

    float kalam;

    for(;;) {

//        IndLED_On(IndColorBlue);

        vTaskDelay(200);
			
			
			

//        ResetStart();

//        CAN_RoboModule_DRV_Position_Mode(0,1,4000,2000*4*15.15);  //2100
//			
//			    Servo1_OPEN;//���1��λ

//        osDelay(3000);

//        CAN_RoboModule_DRV_Position_Mode(0,1,1000,0);
//			
//			    Servo1_CLOSE;//���1��λ

//			osDelay(3000);

//        IndLED_Off();

//        vTaskDelay(1000);



//printf("  tick %u\r\n",HAL_GetTick());
//	printf("  times %llu\r\n",times);
        //		printf("  step_len_throttle %f  step_len_yaw %f  steplen %f\r\n",step_len_throttle,step_len_yaw,RcDetachedParam.detached_params_0.step_length);
        //printf("moto_chassis[0].given_current; %f\r\n",(float)moto_chassis[0].given_current/ 819.2);

//				printf("  step_len_rotate_angle %f   \r\n",step_len_rotate_angle);
//        printf("real_current %f  rev_current %f estimate_moment %f\r\n", moto_chassis[0].real_current, moto_chassis[0].real_current-moto_chassis[0].given_current/ 819.2, moto_chassis[0].real_current*0.3);


        //printf("T; %f\r\n",(float)moto_chassis[0].given_current/ 819.2*0.3);

//        if(ppm_rx[0])//�ɹ�������һ��������
//        {



//           printf("����:%d ǰ��:%d ����:%d ����:%d chanel5-SWA:%d chanel6-SWB:%d chanel7-VRA:%d chanel8-SWC:%d  \r\n",ppm_rx[1],ppm_rx[2],ppm_rx[3],ppm_rx[4],ppm_rx[5],ppm_rx[6],ppm_rx[7],ppm_rx[8]);
//            vTaskDelay(10);
//            ppm_rx[0]=0;
//        }

//        test_speed+=500;
//        IsMotoReadyOrNot= IsReady;
//        vTaskDelay(1500);

//        if(test_speed>=9500)
//        {
//            test_speed=0;
//        }


//temp_pid.ref_agle[0]=0;
//IsMotoReadyOrNot= IsReady;
//        vTaskDelay(2000);

//temp_pid.ref_agle[0]=100000;
//IsMotoReadyOrNot= IsReady;
//				vTaskDelay(2000);

//								if(temp_pid.ref_agle[0]>=8000)
//				{
//					temp_pid.ref_agle[0]=0;
//				}



//StartPosToMiddlePos();

//printf("Pitch %f Roll %f Yaw %f stage %d count %d\r\n",imuinfo.ActVal[1],imuinfo.ActVal[2],imuinfo.ActVal[0],stage,_count_navi);
// printf("_Pitch_rev %f  pitch_offset %f qian%d hou %d\r\n",_Pitch_rev,pitch_offset,HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_2),HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7));

//printf("Roll %f Pitch %f Yaw %f dev_len %f yaw_calibrated %f\r\n",imuinfo.ActVal[2],imuinfo.ActVal[1],imuinfo.ActVal[0],step_len_dev,yaw_calibrated );

//printf("Roll %f Pitch %f Yaw %f\r\n",imuinfo.ActVal[2],imuinfo.ActVal[1],imuinfo.ActVal[0]);

//printf("Yaw %f dev_high %f\r\n",imuinfo.ActVal[0],step_high_dev );






//printf("Yaw %f step_len_dev %f ƫ�Ķ�%f  ���߽�%f  ǰɫ��%f ��ɫ��%f\r\n",imuinfo.ActVal[0],step_len_dev,openmvinfo.ActVal[0],openmvinfo.ActVal[1],openmvinfo.ActVal[2],openmv2info.ActVal[2]);






//		for(int i=0 ; i<8 ; i++)
//		printf(" hall   %d",moto_chassis[i].hall);
//		printf(" \r\n");


//				printf(" CCR1 %d  CCR2 %d  CCR3 %d\r\n",(int)TIM4->CCR1,(int)TIM4->CCR2,(int)TIM4->CCR3);

//        printf("restart1U %d restart2V %d restartclimbc %d restart3W %d	startB %d	inf1S %d inf2T %d\r\n",keyRestart1,keyRestart2,keyRestartclimb,keyRestart3,keyStart,keyInf1,keyInf2);





//        printf("���� %2.1f  ",state_detached_params[state].detached_params_0.stance_height);
//        printf("���� %2.1f  ",state_detached_params[state].detached_params_0.step_length);
//        printf("̧�ȸ� %2.1f  ",state_detached_params[state].detached_params_0.up_amp);
//        printf("ѹ�ȸ� %2.1f  ",state_detached_params[state].detached_params_0.down_amp);
//        printf("����ռ�� %2.2f  ",state_detached_params[state].detached_params_0.flight_percent);
//        printf("Ƶ�� %2.1f  ",state_detached_params[state].detached_params_0.freq);
//        printf("\r\n");





//printf("1 %d 2 %d 3 %d \r\n",USART6RxBuf[0],USART6RxBuf[1],USART6RxBuf[2]);
//kalam=KalmanFilter(imuinfo.ActVal[0],KALMAN_Q,KALMAN_R);
//printf("KEY_VALUE %d Yaw %f  Yaw_Calibrated %f step_len_dev %f",ps2info.KEY_VALUE, imuinfo.ActVal[0],Yaw_Calibrated,step_len_dev);
//printf("KEY_VALUE=%d Yaw %f",ps2info.KEY_VALUE, imuinfo.ActVal[0]);


//		  printf("yaw_set=%f ",yaw_set);
//      printf("\r\n");
//			for(int i=0;i<8;i++)
//			printf("%x ", usart3_buf[i]);
//			printf("\r\n");

//			for(int i=0;i<28;i++)
//			printf("%x ",imu_buf[i]);
//			printf("\r\n");


//			for(int i=0;i<16;i++)
//			printf("%x ",openmv_buf[i]);
//			printf("\r\n");		//�Ƕ�  //�������� // 1�� 2�� 4��





//      printf(" %f  %f  %f\r\n",KalmanFilter(imuinfo.ActVal[2],KALMAN_Q,KALMAN_R),KalmanFilter(imuinfo.ActVal[1],KALMAN_Q,KALMAN_R),KalmanFilter(imuinfo.ActVal[0],KALMAN_Q,KALMAN_R));
//      printf("Roll %f Pitch %f Yaw %f \r\n",imuinfo.ActVal[2],imuinfo.ActVal[1],imuinfo.ActVal[0]);
//      printf(" %x %x %x  %x %x %x %x %x %x\n",dbus_buf[0],dbus_buf[1],dbus_buf[2],dbus_buf[3],dbus_buf[4],dbus_buf[5],dbus_buf[6],dbus_buf[7],dbus_buf[8]);

//		mpu_get_data();
//		imu_ahrs_update();
//		imu_attitude_update();
//		HAL_Delay(5);
//		printf(" Roll:  \n");
//		HAL_UART_Transmit(&huart6, (uint8_t *)buf, (COUNTOF(buf)-1), 55);
//		HAL_Delay(5);

//		Servo_DOWN;
//		wave_form_data[1] =count;
//		wave_form_data[2] =count;
//    temp_pid.ref_agle[0]-=30.0f*ReductionAndAngleRatio;
//    temp_pid.ref_agle[1]+=30.0f*ReductionAndAngleRatio;

//		temp_pid.ref_agle[0]=count*TransData;
//		count+=10;
//		if(count>=50)
//		{count=0;}

//		printf(" ref_agle[0]  = %f   ref_agle[1]  =%f  \n",temp_pid.ref_agle[0],temp_pid.ref_agle[1]);
//		printf(" theat1  = %f   theat2  =%f  \n",theta1,theta2);

//		Servo_PEAK;

    }
}


void ResetStart(void)
{

    if(keyRestart3==0) {
        while(keyRestart3==0)
            vTaskDelay(200);

        CAN_RoboModule_DRV_Position_Mode(0,1,2000,0);

        restartflag = 1;

				IndicateLED_Off;
        IndLED_On(IndColorRed);
				IndLED_On(IndColorBlue);
        state= STOP;
        vTaskDelay(300);
        vTaskDelete(LogicalFlowTask_Handler);
        vTaskDelay(200);

        taskENTER_CRITICAL();           //�����ٽ���
        xTaskCreate((TaskFunction_t )LogicalFlow_task,
                    (const char*    )"LogicalFlow_task",
                    (uint16_t       )LogicalFlow_STK_SIZE,
                    (void*           )NULL,
                    (UBaseType_t    )LogicalFlow_TASK_PRIO,
                    (TaskHandle_t*  )&LogicalFlowTask_Handler);
        taskEXIT_CRITICAL();            //�˳��ٽ���


       
          IndicateLED_Off;
    }


}

void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
static void MX_NVIC_Init(void)
{
    /* USART1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USART2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* USART3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* USART6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    /* USART7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);

    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);


}

void Error_Handler(void)
{
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
