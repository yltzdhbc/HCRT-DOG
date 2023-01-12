#ifndef BUZZER_H
#define BUZZER_H
#include "robocon.h"

void buzzer_init(uint16_t arr, uint16_t psc);
void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void promising_young(void);
void BeginWarnBuzzer(void);
void IMUWarnBuzzer(void);
void happy_time(void);
void ActionDoneBuzzer(void);
void ProcessBuzzer(void);

extern TIM_HandleTypeDef TIM12_Handler;
extern TIM_OC_InitTypeDef TIM12_CH1Handler;

#define  proport          100000 //Tclk/(psc+1)=180000000/(89+1)
#define  L1       ((proport/131)-1)//�͵���do �����ڸ���Tout= ((arr+1)*(psc+1))/Tclk�Ƴ�arrֵ���Ǳ���define�����ֵ��ToutΪ����Ƶ��131Hz�ĵ�����Tclk=72MHz
#define  L2       ((proport/147)-1)//�͵���re ������
#define  L3       ((proport/165)-1)//�͵���mi ������
#define  L4       ((proport/176)-1)//�͵���fa ������
#define  L5       ((proport/196)-1)//�͵���sol������
#define  L6       ((proport/220)-1)//�͵���la ������
#define  L7       ((proport/247)-1)//�͵���si ������

#define  M1       ((proport/262)-1)//�е���do ������
#define  M2       ((proport/296)-1)//�е���re ������
#define  M3       ((proport/330)-1)//�е���mi ������ 
#define  M4       ((proport/349)-1)//�е���fa ������
#define  M5       ((proport/392)-1)//�е���sol������
#define  M6       ((proport/440)-1)//�е���la ������
#define  M7       ((proport/494)-1)//�е���si ������

#define  H1       ((proport/523)-1)//�ߵ���do ������
#define  H2       ((proport/587)-1)//�ߵ���re ������
#define  H3       ((proport/659)-1)//�ߵ���mi ������
#define  H4       ((proport/699)-1)//�ߵ���fa ������
#define  H5       ((proport/784)-1)//�ߵ���sol������
#define  H6       ((proport/880)-1)//�ߵ���la ������
#define  H7       ((proport/988)-1)//�ߵ���si ������

#define  Z0       0//

//int length = sizeof(music)/sizeof(music[0]);//�������鳤��

#endif
