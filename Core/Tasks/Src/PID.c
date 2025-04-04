/*
 * PID.c
 *
 *  Created on: Mar 7, 2025
 *      Author: Admin
 */




/****************PID����****************/

#include "./PID.h"

//��ʼ��pid����
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->deadzone=0;
	pid->errLpfRatio=1;
}

//����pid����
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//��������
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//���������������errorֱ����0
		pid->error=0;
	else
		pid->error=reference-feedback;
	//��ͨ�˲�
	pid->error=pid->error*pid->errLpfRatio+pid->lastError*(1-pid->errLpfRatio);
	//����΢��
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//�������
	pid->output+=pid->error*pid->kp;
	//�������
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//�����޷�
	pid->output+=pid->integral;
	//����޷�
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//����pid����
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PID_SingleCalc(&pid->outer,angleRef,angleFdb);//�����⻷(�ǶȻ�)
	PID_SingleCalc(&pid->inner,pid->outer.output,speedFdb);//�����ڻ�(�ٶȻ�)
	pid->output=pid->inner.output;
}

//���һ��pid����ʷ����
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//�����趨pid����޷�
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	pid->maxOutput=maxOut;
}

//����PID����
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}

//����LPF����
void PID_SetErrLpfRatio(PID *pid,float ratio)
{
	pid->errLpfRatio=ratio;
}
