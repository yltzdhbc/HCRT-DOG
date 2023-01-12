
#include "servo.h"

TIM_HandleTypeDef TIM4_Handler;
TIM_OC_InitTypeDef TIM4_CH1Handler;
TIM_OC_InitTypeDef TIM4_CH2Handler;
TIM_OC_InitTypeDef TIM4_CH3Handler;

void Servo_Init(uint16_t arr, uint16_t psc)
{

    TIM4_Handler.Instance=TIM4;            //��ʱ��
    TIM4_Handler.Init.Prescaler=psc;       //��ʱ����Ƶ
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM4_Handler.Init.Period=arr;          //�Զ���װ��ֵ
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM4_Handler);       //��ʼ��PWM

    TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM4_CH1Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM4_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ��
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);//����PWMͨ��1

    TIM4_CH2Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM4_CH2Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM4_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ��
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH2Handler,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_2);//����PWMͨ��2

    TIM4_CH3Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM4_CH3Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM4_CH3Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ��
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH3Handler,TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_3);//����PWMͨ��3

}
