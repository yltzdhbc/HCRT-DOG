
#include "buzzer.h"

TIM_HandleTypeDef TIM12_Handler;
TIM_OC_InitTypeDef TIM12_CH1Handler;


void ProcessBuzzer(void)
{

    int sound[]=
    {
        M7,750,Z0,

    };

    int length = sizeof(sound)/sizeof(sound[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
    buzzer_off();

}



void ActionDoneBuzzer(void)
{

    int sound[]=
    {
        M7,50,Z0,10,M7,50,Z0,10,M7,100,Z0,10,

    };

    int length = sizeof(sound)/sizeof(sound[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
    buzzer_off();

}


/**
* NAME: void BeginWarnBuzzer(void)
* FUNCTION : ���������� do mi xi
*/
void BeginWarnBuzzer(void)
{
    int sound[]=
    {
        M1,50,M3,50,M7,100,Z0,150,
        // H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,100, H3,150,Z0,25,
    };

    int length = sizeof(sound)/sizeof(sound[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }

}

/**
* NAME: void IMUWarnBuzzer(void)
* FUNCTION : IMUУ׼��ʾ��
*/
void IMUWarnBuzzer(void)
{
    int sound[]=
    {
        M7,50,Z0,150,
        H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,25, H3,25,Z0,100, H3,150,Z0,25,
    };

    int length = sizeof(sound)/sizeof(sound[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(sound[2*i], 200);
        vTaskDelay(4*sound[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }

}

/**
* NAME: void happy_time(void)
* FUNCTION : happy
*/
void happy_time(void)
{
    int happy_birthday[]=
    {
        M5,50,M5,25,M5,25,
        M6,100,M5,100,H1,100,
        M7,100,M7,100,M5,50,M5,25,M5,25,
        M6,100,M5,100,H2,100,
        H1,100,H1,100,M5,50,M5,25,M5,25,
        H5,100,H3,100,H1,100,
        M7,100,M6,100,H4,50,H4,25,H4,25,
        H3,100,H1,100,H2,100,H1,100,H1,100
    };

    int length = sizeof(happy_birthday)/sizeof(happy_birthday[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(happy_birthday[2*i], 200);
        vTaskDelay(5*happy_birthday[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}

/**
* NAME: void promising_young(void)
* FUNCTION : promising_young
*/
void promising_young(void)
{
    int promising_young[]=
    {
        M5,50,M6,50,H1,50,Z0,10,H2,50,   Z0,10,H2,50,Z0,1,H2,50, Z0,1, H1,50, Z0,1,H2,50,   Z0,15, H1,50, Z0,15,H3,50,  Z0,15,Z0,50,
        M7,40,Z0,10,M7,40,Z0,10,M7,40,Z0,10,M7,75,Z0,15,M7,50, Z0,25, M6,25, Z0,25, H1,25,Z0,25

    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}

void buzzer_init(uint16_t arr, uint16_t psc)
{

    TIM12_Handler.Instance=TIM12;            //��ʱ��
    TIM12_Handler.Init.Prescaler=psc;       //��ʱ����Ƶ
    TIM12_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM12_Handler.Init.Period=arr;          //�Զ���װ��ֵ
    TIM12_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM12_Handler);       //��ʼ��PWM

    TIM12_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM12_CH1Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM12_CH1Handler.OCPolarity=TIM_OCPOLARITY_LOW; //����Ƚϼ���Ϊ��
    HAL_TIM_PWM_ConfigChannel(&TIM12_Handler,&TIM12_CH1Handler,TIM_CHANNEL_1);//����TIM12ͨ��1

    HAL_TIM_PWM_Start(&TIM12_Handler,TIM_CHANNEL_1);//����PWMͨ��1

    buzzer_off();

}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    TIM12->PSC = psc;
    TIM12->CCR1=pwm;

}

void buzzer_off(void)
{
    TIM12->CCR1=0;
}

