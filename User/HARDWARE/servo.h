#ifndef __SERVO_H
#define __SERVO_H
#include "robocon.h"
void Servo_Init(uint16_t arr, uint16_t psc);

extern TIM_HandleTypeDef TIM4_Handler;
extern TIM_OC_InitTypeDef TIM4_CH1Handler;
extern TIM_OC_InitTypeDef TIM4_CH2Handler;

#define Servo1_PEAK TIM4->CCR1=1020
#define Servo1_DOWN TIM4->CCR1=2040


#define Servo2_PEAK_POS TIM4->CCR2=1320
#define Servo2_DOWN_POS TIM4->CCR2=2360


#define Servo1_TEST_POSITION TIM4->CCR1=1620
#define Servo2_TEST_POSITION TIM4->CCR2=2060


#endif
