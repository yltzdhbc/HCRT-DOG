/**
*	����mti30�ĺ��� ����һ������ DMA����
* ע��mti30���� �ߵ�λ�Ƿ��� ��Ҫ�ȷ�ת���� ���ù�����bsp_can�����Ѿ���ɣ�
  */


#include "mti30.h"

//Ԥ�� ******
/*Ŀǰ�Ĵ�����bsp_can���� 
*��������Ļ�
*������ֲ���� 
*��ý�֮�ŵ�����д
*/

/**************************************һά�������˲���**************************************/
/*       
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��       
*/

/* �������˲����� */

float KalmanFilter( float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{

    float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;

    static float x_last;
    float x_mid = x_last;
    float x_now;

    static float p_last;
    float p_mid ;
    float p_now;

    float kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����

    /*
     *  �������˲��������Ҫ��ʽ
     */
    kg=p_mid/(p_mid+R);                 //kgΪkalman filter��R Ϊ����
    x_now=x_mid+kg*(ResrcData-x_mid);   //���Ƴ�������ֵ
    p_now=(1-kg)*p_mid;                 //����ֵ��Ӧ��covariance
    p_last = p_now;                     //����covariance ֵ
    x_last = x_now;                     //����ϵͳ״ֵ̬

    return x_now;

}

