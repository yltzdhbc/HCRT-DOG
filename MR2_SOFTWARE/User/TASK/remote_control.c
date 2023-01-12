/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң����ͨ��DMA���ڽ������ݣ����ô��ڿ����ж�������
  *            	����
  ==============================================================================
  **************************** HBUT ROBOCON 2019****************************
  */

#include "Remote_Control.h"
extern TaskHandle_t LogicalFlowTask_Handler;

float step_len_throttle;
float step_len_throttle_yaw;
float rc_params_l,rc_params_r;

float step_len_rotate_angle;

bool StartFlag = 0;

void rc_params_clean(void)
{
    step_len_throttle=0;
    step_len_throttle_yaw=0;
    step_len_rotate_angle=0;
    state_gait_params[ROTAT_LEFT].step_length=0;
    RcDetachedParam.detached_params_0.step_length=0;
    RcDetachedParam.detached_params_2.step_length=0;
    RcDetachedParam.detached_params_1.step_length=0;
    RcDetachedParam.detached_params_3.step_length=0;

}

void Rc_task(void *pvParameters)
{

//    while(!imu_rec_flag)		//�ȴ�ACTION��ʼ��
//    {
//        IndLED_On(IndColorGreen);
//        vTaskDelay(50);
//        IndLED_Off();
//        vTaskDelay(50);
//    }

//    IndLED_On(IndColorRed);

//    ActionDoneBuzzer();

    for(;;)
    {
        //----------------------��������---���ذ���-------------------//
        if(keyStart==0&&StartFlag == 0)  //599�� 1599��
        {
            while(keyStart==0)
                vTaskDelay(20);
            StartPosToMiddlePos();
            StartFlag=1;
            IndicateLED_Off;
            IndLED_On(IndColorGreen);
        }
        else if(keyStart==0&&StartFlag == 1)
        {
            while(keyStart==0)
                vTaskDelay(20);
            CAN_RoboModule_DRV_Position_Mode(0,1,2000,0);
            MiddlePosToEndPos();
            StartFlag=0;
        }

//----------------------ң������-------------------------------------------------//

//        if(ppm_rx[7]>590&&ppm_rx[7]<1600)  //VRA    ѡ���Ƿ���ң�ؿ���    600 850 1100 1350 1600
//        {
//            rc_ctrl_flag = 1;


     
        //----------------------��----------------------//
        if(ppm_rx[5]>1500)  //599�� 1599��
        {
            while(ppm_rx[5]>1500)
                vTaskDelay(200);
            state = REALSE;
            StartJump(HAL_GetTick());
						vTaskDelay(2000);
            printf("JUMP\r\n");
        }

				
				
				
            //------------------------����--------------------------//
            if(ppm_rx[3]>590&&ppm_rx[3]<660) { //599�� 1599��  ����

                if(ppm_rx[4]>600&&ppm_rx[4]<900) {  //���
                    step_len_rotate_angle=((float)(-(ppm_rx[4]-1100+200)))/100*4;
                    state=ROTAT_LEFT;
                    state_gait_params[ROTAT_LEFT].step_length=step_len_rotate_angle;
                    vTaskDelay(50);
                }

                else if(ppm_rx[4]>1300&&ppm_rx[4]<1600) {//�Ҵ�
                    step_len_rotate_angle=(float)(ppm_rx[4]-1100-200)/100*4;
                    state=ROTAT_RIGHT;
                    state_gait_params[ROTAT_RIGHT].step_length=step_len_rotate_angle;
                    vTaskDelay(50);
                }

                else {
                    rc_params_clean();
                    state = REALSE;
                    vTaskDelay(5);
                }
            }
            else if(ppm_rx[3]>=660&&ppm_rx[3]<800) {
                rc_params_clean();
                state = STOP;
                vTaskDelay(5);
            }
            else if(ppm_rx[3]>=800&&ppm_rx[3]<900) {
                rc_params_clean();
                state = TROT;
                vTaskDelay(5);
            }
            else if(ppm_rx[3]>=900&&ppm_rx[3]<1600) {
                vTaskDelay(5);

                step_len_throttle=((float)ppm_rx[3]-900)/100*3; //0-21
                step_len_throttle_yaw=((float)ppm_rx[1]-1099)/100*3;

                rc_params_l=step_len_throttle+step_len_throttle_yaw;
                rc_params_r=step_len_throttle-step_len_throttle_yaw;
                if(rc_params_l<0) rc_params_l=0;
                if(rc_params_r<0) rc_params_r=0;

                RcDetachedParam.detached_params_0.step_length=rc_params_l;
                RcDetachedParam.detached_params_2.step_length=rc_params_l;
                RcDetachedParam.detached_params_1.step_length=rc_params_r;
                RcDetachedParam.detached_params_3.step_length=rc_params_r;
            }






        vTaskDelay(100);

    }


}
