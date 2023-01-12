/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       logicalflow_task.c/h
  * @brief
  ==============================================================================
  **************************** HBUT ROBOCON 2019****************************
  */

#include "logicalflow_task.h"

//		RED_GROUNG		BLUE_GROUNG

bool GROUND_SELECT=RED_GROUNG;  //������ѡ��

bool restartflag = 0;

/*********�������˿�˵��********************
*
*���˵����1�ţ����˶����H:PD12		2�ţ����ƶ����G:PD13		3�ţ����������F:PD14
*
*��λ����		����1 X:PI6		����2 A:PI0		����CLIMB C:PH11  ����STA W:PI5		����B:PH12
*��翪�أ����ֿ�ʼ���£����1 S:PA0 ���2 T:PA1
*
*ָʾLED Z PI2
**/

/**
*		���������������߼�����
*/
void LogicalFlow_task(void *pvParameters)
{
    for(;;) {

//        CAN_RoboModule_DRV_Position_Mode(0,1,2000,2150*4*15.15);  //�����Լ�
//        osDelay(2000);
//        CAN_RoboModule_DRV_Position_Mode(0,1,2000,0);

        while(1)
        {
            vTaskDelay(100);
            if(keyRestart1==0) {//����1 U:PA2
                while(keyRestart1==0)
                    vTaskDelay(20);

                IndicateLED_Off;
                IndLED_On(IndColorBlue);
                IndLED_On(IndColorRed);
                goto restart1;
            }
            else if(keyRestart2==0) { //����2 V:PA3
                while(keyRestart2==0)
                    vTaskDelay(20);

                IndicateLED_Off;
                IndLED_On(IndColorBlue);
                IndLED_On(IndColorGreen);
                goto restart2;
            }
            else if(keyRestartclimb==0) //����climb
            {
                while(keyRestartclimb==0)
                    vTaskDelay(20);

                IndicateLED_Off;
                IndLED_On(IndColorRed);
                IndLED_On(IndColorGreen);
                state = STOP;
                goto restartclimb;
            }

            if(StartFlag == 1&&restartflag == 0) {
                osDelay(100);
                break;
            }
        }


        while((keyInf2!=0)) //�ȴ����ƹ�翪�ش���
            vTaskDelay(500);

        IndLED_Off();

        IndLED_On(IndColorRed);
        vTaskDelay(1500);  				//�ȴ� mr1��
        Servo1_CLOSE;


        pid_spd_out_limit=  2000;

        state = STOP;
        vTaskDelay(800);
        IndLED_Off();

        pid_spd_out_limit=  6000;

        yaw_set=0;  //------������趨Ϊ0
        LinearCorrection=normal_correction;		//��ֱ�߽���

        state = TROT;		//С�ܲ�̬

        vTaskDelay(2000);  				//�ȴ� ����⵽��ɫ��������

        OpenMvInspect(openmv_Red);  //��⵽��ɫ  ��ʼת��



        IndLED_On(IndColorRed);  //���ָʾ


        if(GROUND_SELECT==RED_GROUNG) {//�������ж� �쳡
            yaw_set=10;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]<=(_rotate_angle-4))
                osDelay(50);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=-10;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]>=(_rotate_angle+4))
                osDelay(50);
        }

        IndLED_Off(); //ָʾ����
        OpenMvInspect(openmv_Yellow);  // -------- �ȴ���⵽ת�� ��ɫ --ɳ��

				
				restart1:
        /////restart only
        pid_spd_out_limit=  6000;
       // yaw_set=0;  //------������趨Ϊ0
        LinearCorrection=normal_correction;		//��ֱ�߽���
        state = TROT;		//С�ܲ�̬
        /////------end
				
				vTaskDelay(1000);
				
				
				
        if(GROUND_SELECT==RED_GROUNG) {//�������ж� �쳡
            yaw_set=10;
            _rotate_angle=yaw_set;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=_rotate_angle)
                osDelay(20);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=-10;
            _rotate_angle=yaw_set;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=_rotate_angle)
                osDelay(20);
        }


        //////////////Խɳ��//////////////

        IndLED_On(IndColorRed);

        state = STOP;
        vTaskDelay(200);

        state = REALSE;
        vTaskDelay(300);

        pid_spd_out_limit=  6700;

        StartJump(HAL_GetTick()); //��һ������
        vTaskDelay(2300);

        state = REALSE;
        StartJump(HAL_GetTick());	//�ڶ�������
        vTaskDelay(2300);

        pid_spd_out_limit=  6000;

        IndLED_Off(); //ָʾ����

        //////////////Խɳ�����//////////////



        pid_spd_out_limit=  6000;

        //////////Խ��̨��֮���ֱ�����߲���
        state=TROT;
        if(GROUND_SELECT==RED_GROUNG) {  //�������ж� �쳡
            yaw_set=14.5;
            _rotate_angle=yaw_set;
            LinearCorrection=normal_correction;		//��ֱ�߽���
            while(imuinfo.ActVal[0]<=(_rotate_angle-1.1))  //����ת����
                osDelay(50);

            yaw_set=13.6;  //�趨�Ƕ�Ϊ13
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=-14.5;
            _rotate_angle=yaw_set;
            LinearCorrection=normal_correction;		//��ֱ�߽���
            while(imuinfo.ActVal[0]>=(_rotate_angle+1.1))
                osDelay(50);

            yaw_set=-13.6;
        }
        //////////ֱ�����߲�����



        OpenMvInspect(openmv_Yellow); //�ȴ���⵽��ɫ ɫ��----------Խ���Ӽ��
        IndLED_On(IndColorRed);  //ָʾ�ƿ�


        //////�����ɫ����֮��ת�䶯��
        if(GROUND_SELECT==RED_GROUNG) {  //�������ж� �쳡
            yaw_set=0;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]>=(_rotate_angle+5.5))
                osDelay(20);
        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=0;
            _rotate_angle=yaw_set;
            while(imuinfo.ActVal[0]<=(_rotate_angle-5.5))
                osDelay(20);
        }
        IndLED_Off();


        if(GROUND_SELECT==RED_GROUNG) {  //�������ж� �쳡
            yaw_set=0;
            _rotate_angle=yaw_set;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=(_rotate_angle+0.5))
                osDelay(20);

        }
        else if(GROUND_SELECT==BLUE_GROUNG) {
            yaw_set=0;
            _rotate_angle=yaw_set;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=(_rotate_angle-0.5))
                osDelay(20);

        }
        //////ת�䶯������


restart2:

        pid_spd_out_limit=  6000;
        state=TROT;
        yaw_set=0.0;
        LinearCorrection=normal_correction;		//��ֱ�߽���
        OpenMvInspect(openmv_Yellow);

				
        state=STOP;
        vTaskDelay(500);

//				state=WALK_BACK;
//				 vTaskDelay(1000);

//        state=STOP;
//        vTaskDelay(500);


        ////�����������
        pid_spd_out_limit=  6700;

        jump_angle=45.5;
        jump_extension=28.8;

        state = REALSE;
        StartJump(HAL_GetTick());	//��һ������
        vTaskDelay(2200);


        //////////��һ������֮������Լ�����̬
        if(imuinfo.ActVal[0]>=0) {
            yaw_set=0-0;
            _rotate_angle=yaw_set;
            state = ROTAT_RIGHT;
            while(imuinfo.ActVal[0]>=(_rotate_angle+1.2))
                osDelay(10);
        }
        else if(imuinfo.ActVal[0]<=0) {
            yaw_set=0-0;
            _rotate_angle=yaw_set;
            state = ROTAT_LEFT;
            while(imuinfo.ActVal[0]<=(_rotate_angle-1.2))
                osDelay(10);
        }
        //////////��̬��������


        state = REALSE;
        StartJump(HAL_GetTick());	//�ڶ�������
        vTaskDelay(2100);

        state = REALSE;
        StartJump(HAL_GetTick());	//�����δ�����
        vTaskDelay(2100);


        state = REALSE;
        StartJump(HAL_GetTick());	//�����δ�����
        vTaskDelay(2100);


        pid_spd_out_limit=  6700;


        //�����Ӵ�ͳ��̬   ʱ��20s
        //	CrossTheLine();
        //	yaw_set=0-0; //������趨 0 ����-0
//        	state=TEST1;
//        	vTaskDelay(1000);


restartclimb:

        state= STOP;

        //////////����
        Climbing_Comb();
        //////////�������

        while(1)		//ͣ
            vTaskDelay(500);


    }
}
