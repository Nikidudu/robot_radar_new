/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     �������ǶԻ����˶��ٶȹ��ƣ��������ƴ�
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "kalman_filter.h"
#include "board_lib.h"
//#include "cmsis_os.h"

KalmanFilter_t vaEstimateKF;	   // �������˲����ṹ��

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
                           0.0f, 0.5f};    // Q�����ʼֵ

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// ���þ���HΪ����
														 															 
extern INS_t INS;		
//extern chassis_t chassis_move;
																 
//extern vmc_leg_t right;
//extern vmc_leg_t left;

float vel_acc[2]; 
uint32_t OBSERVE_TIME=5;//����������3ms
void 	Observe_task(void)
{
	while(INS.ins_flag==0)
	{//�ȴ����ٶ�����
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
		
	xvEstimateKF_Init(&vaEstimateKF);
	
  while(1)
	{  
		wr= -chassis_move.wheel_motor[0].para.vel-INS.Gyro[0]+right.d_alpha;//�ұ�������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
		vrb=wr*0.0603f+right.L0*right.d_theta*arm_cos_f32(right.theta)+right.d_L0*arm_sin_f32(right.theta);//����bϵ���ٶ�
		
		wl= -chassis_move.wheel_motor[1].para.vel+INS.Gyro[0]+left.d_alpha;//���������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
		vlb=wl*0.0603f+left.L0*left.d_theta*arm_cos_f32(left.theta)+left.d_L0*arm_sin_f32(left.theta);//����bϵ���ٶ�
		
		aver_v=(vrb-vlb)/2.0f;//ȡƽ��
    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_n[1],aver_v);
		
		//ԭ����ת�Ĺ�����v_filter��x_filterӦ�ö���Ϊ0
		chassis_move.v_filter=vel_acc[0];//�õ��������˲�����ٶ�
		chassis_move.x_filter=chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
	//�����ֱ���������ٶȣ������ںϵĻ���������
	//chassis_move.v_filter=(chassis_move.wheel_motor[0].para.vel-chassis_move.wheel_motor[1].para.vel)*(-0.0603f)/2.0f;//0.0603�����Ӱ뾶������������ǽ��ٶȣ��˰뾶��õ����ٶȣ���ѧģ���ж����������˳ʱ��Ϊ��������Ҫ�˸�����
	//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// ״̬����2ά û�п����� ��������2ά
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //�������˲�������ֵ����
    EstimateKF->MeasuredVector[0] =	vel;//�����ٶ�
    EstimateKF->MeasuredVector[1] = acc;//�������ٶ�
    		
    //�������˲������º���
    Kalman_Filter_Update(EstimateKF);

    // ��ȡ����ֵ
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}


