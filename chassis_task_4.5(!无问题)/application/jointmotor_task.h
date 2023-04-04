///**
//  ****************************(C) COPYRIGHT 2022 SuperPower****************************
//  * @file       jointmotor_task.c/h
//  * @brief      joint motor control task,
//  *             �ؽڵ����������
//  * @note       
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.0     Apr-04-2022     HYX              1. done
//  @verbatim
//  ==============================================================================

//  ==============================================================================
//  @endverbatim
//  ****************************(C) COPYRIGHT 2022 SuperPower****************************
//  */

#ifndef JOINTMOTOR_TASK_H
#define JOINTMOTOR_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//Y�����˶���KP��KDֵ
#define KP_Y 100.0f
#define KD_Y 40.0f

//ROLL�����˶���KP��KDֵ
#define KP_ROLL 20.0f
#define KD_ROLL 5.0f

//�����ʼ�����ӳ�ʱ��
#define JOINTMOTOR_TASK_INIT_TIME 900

//�������������ʱ�䣬ms
#define JOINTMOTOR_CONTROL_TIME_MS 5

//�������������ʱ�䣬s
#define JOINTMOTOR_CONTROL_TIME_S 0.005

////�ؽڵ������λ�Ƕȣ�150�㻻��Ϊ������
//#define INIT_ANGLE 2.618

//�Ȳ����˵ĳ��Ȳ���
#define l1 0.1
#define l2 0.16
#define l3 0.32

//����y�����Roll�����ҡ��ͨ����
#define LEG_Y_CHANNEL 1
#define LEG_ROLL_CHANNEL 0

//ҡ��λ������ӳ�䵽y����߶Ⱥ�Roll����Ƕȵı���ϵ��
#define LEG_Y_RC_SEN 0.0002
#define LEG_ROLL_RC_SEN 0

//y������м�߶�
#define LEY_Y_MID_HEIGHT 0.27

//��߻���������6kg��9.8=58.8��
#define G 15

//���ּ��
#define D 0.473

////�ؽڵ�����ݽṹ��
//typedef struct
//{
//  const joint_motor_measure_t *joint_motor_measure;
//	uint32_t joint_motor_id;    //�ؽڵ��ID
//	fp32 angle;                 //�ؽڵ���Ƕȣ���λrad
//	fp32 torque_set;            //�ؽڵ�������趨ֵ
//} joint_motor_t;

//�Ȳ��˶����ݽṹ��
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //��ȡң����ָ��
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  const fp32 *chassis_INS_angle_speed;       //��ȡ�����ǽ��������ת���ٶ�ָ��
	const fp32 *accel_fliter;                  //��ȡ�˲���ļ��ٶȼ�����ָ��
//	joint_motor_t joint_motor[4];              //�ؽڵ������
	
	fp32 y;                                    //����߶�
	fp32 yL;                                   //����Ȳ��߶�
	fp32 yR;                                   //�ұ��Ȳ��߶�
	fp32 delta_y;                              //����߶��趨ֵ�뷴��ֵ�Ĳ�ֵ
	fp32 delta_roll;                           //����roll�Ƕ��趨ֵ�뷴��ֵ�Ĳ�ֵ
	fp32 y_set;                                //����߶��趨ֵ
	fp32 roll_set;                             //ROLL��Ƕ��趨ֵ
	fp32 vy;                                   //����y������ٶ�
	fp32 last_vy;
	fp32 vyL;                                  //����Ȳ���λ��΢��ֵ����������ĩ��ִ����ת���ɹؽڵ������
	fp32 vyR;                                  //�ұ��Ȳ���λ��΢��ֵ����������ĩ��ִ����ת���ɹؽڵ������
	fp32 FL;                                   //����Ȳ���ĩ��ִ����
	fp32 FR;                                   //�ұ��Ȳ���ĩ��ִ����
	fp32 y_accel;                              //�˲����y������ٶ�
	
	fp32 chassis_yaw;                          //���������Ƿ����ĵ�ǰyaw�Ƕ�
  fp32 chassis_pitch;                        //���������Ƿ����ĵ�ǰpitch�Ƕ�
  fp32 chassis_roll;                         //���������Ƿ����ĵ�ǰroll�Ƕ�
  fp32 chassis_yaw_speed;                    //���������Ƿ����ĵ�ǰyaw���ٶ�
	fp32 chassis_pitch_speed;                  //���������Ƿ����ĵ�ǰpitch���ٶ�
	fp32 chassis_roll_speed;                   //���������Ƿ����ĵ�ǰroll���ٶ�
}leg_move_t;

extern void jointmotor_task(void const *argument);

#endif