/**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       jointmotor_task.c/h
  * @brief      joint motor control task,
  *             �ؽڵ����������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-04-2022     HYX              1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  */

#include "jointmotor_task.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "detect_task.h"
#include "bsp_delay.h"
#include "arm_math.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "motor_control.h"
//void MotorControl_Start(leg_move_t *leg_move_start);
//void MotorControl_Stop(leg_move_t *leg_move_stop);
void jointmotor_init(leg_move_t *leg_move_init);
static void jointmotor_feedback_update(leg_move_t *leg_move_update);
static void jointmotor_control_loop(leg_move_t *leg_move_control);
static void leg_set_control(leg_move_t *leg_set);

////�Ȳ��˶�����
leg_move_t leg_move;

MOTOR_send left_byd_send;
MOTOR_recv left_byd_recv;
MOTOR_send left_frt_send;
MOTOR_recv left_frt_recv;

MOTOR_send right_byd_send;
MOTOR_recv right_byd_recv;
MOTOR_send right_frt_send;
MOTOR_recv right_frt_recv;

float T1;
float T2;
float T3;
float T4;
/**
  * @brief          joint motor task, osDelay JOINTMOTOR_CONTROL_TIME_MS (5ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �ؽڵ���������񣬼�� JOINTMOTOR_CONTROL_TIME_MS 5ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void jointmotor_task(void const *argument)
{
	//wait a time 
	//����һ��ʱ��
	vTaskDelay(JOINTMOTOR_TASK_INIT_TIME);
	jointmotor_init(&leg_move);
	
	right_byd_send.T = -0;
	left_byd_send.T = 0;
	while(1)
	{
		//joint motor data update
		//�ؽڵ�����ݸ���		
		jointmotor_feedback_update(&leg_move);
		//set joint motor control set-point 
		//�ؽڵ������������
		leg_set_control(&leg_move);		
		//joint motor control value calculate
		//�ؽڵ������������
		jointmotor_control_loop(&leg_move);
		
		left_joint_motor_control(&left_byd_send,&left_byd_recv);
		left_joint_motor_control(&left_frt_send,&left_frt_recv);
		right_joint_motor_control(&right_byd_send,&right_byd_recv);
		right_joint_motor_control(&right_frt_send,&right_frt_recv);
		vTaskDelay(JOINTMOTOR_CONTROL_TIME_MS);
	}
}


/**
  * @brief          "leg_move" valiable initialization, include remote control data point initialization, gyro sensor angle point and angular velocity point initialization,
  *                 HT03 joint motors data point initialization, accel fliter data point initialization.
  * @param[out]     leg_move_init: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"leg_move"����������ң����ָ���ʼ���������ǽǶȺͽ��ٶ�ָ���ʼ����HT03�ؽڵ��ָ���ʼ�������ٶȼ��˲�����ָ���ʼ��
  * @param[out]     leg_move_init:"leg_move"����ָ��.
  * @retval         none
  */
void jointmotor_init(leg_move_t *leg_move_init)
{
	if(leg_move_init == NULL )
	{
		return ;
	}
	
  //get remote control point
  //��ȡң����ָ��
	leg_move_init->chassis_RC = get_remote_control_point();
	
	//get gyro sensor euler angle point
  //��ȡ��������̬��ָ��
	leg_move_init->chassis_INS_angle = get_INS_angle_point();
	
	//get gyro sensor euler angular velocity
	//��ȡ�����ǽ��ٶ�ָ��
  leg_move_init->chassis_INS_angle_speed = get_gyro_data_point();
	//get accel fliter data
	//��ȡ���ٶȼ��˲��������
	leg_move_init->accel_fliter = get_accel_fliter_data_point();
	
	//initialize all joint motors ID
	//��ʼ�����йؽڵ��ID
	left_byd_send.mode = 10;
	left_frt_send.mode = 10;
	right_byd_send.mode = 10;
	right_frt_send.mode = 10;
	
	left_byd_send.id = 0;
	left_frt_send.id = 1;
	right_byd_send.id = 0;
	right_frt_send.id = 1;
	
	left_byd_send.W = 0;
	left_byd_send.Pos=0;
	left_byd_send.K_P=0;
	left_byd_send.K_W=0; 
	
	left_frt_send.W = 0;
	left_frt_send.Pos=0;
	left_frt_send.K_P=0;
	left_frt_send.K_W=0; 

	right_byd_send.W = 0;
	right_byd_send.Pos=0;
	right_byd_send.K_P=0;
	right_byd_send.K_W=0; 
	
	right_frt_send.W = 0;
	right_frt_send.Pos=0;
	right_frt_send.K_P=0;
	right_frt_send.K_W=0; 
	
	//������y������ٶȳ�ʼ��Ϊ0
	leg_move_init->vy  = 0;
	//update data
	//����һ������
  jointmotor_feedback_update(leg_move_init);
}

/**
  * @brief          joint motor some measure data updata, such as motor angle, robot euler angle and angular velocity��leg height��velocity of y-axis
  * @param[out]     leg_move_update: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          �ؽڵ���������ݸ��£���������Ƕȣ�������ŷ���ǶȺͽ��ٶȣ��Ȳ��߶ȣ�y�����ٶ�
  * @param[out]     leg_move_update:"leg_move"����ָ��.
  * @retval         none
  */
static void jointmotor_feedback_update(leg_move_t *leg_move_update)
{
	if(leg_move_update == NULL)
	{
		return;
	}

	//�ؽڵ���Ƕ�ֵ����
	right_frt_recv.jointAngle = 2.617 + 0.42 - right_frt_recv.Pos;
	left_frt_recv.jointAngle = 2.617 - 0.282 + left_frt_recv.Pos;

	//�����Ȳ��߶��Լ�����߶ȸ���
	leg_move_update->yR = sqrt(pow(l3,2)-pow(l1-l2*cos(right_frt_recv.jointAngle),2))-l2*sin(right_frt_recv.jointAngle);
	leg_move_update->yL = sqrt(pow(l3,2)-pow(l1-l2*cos(left_frt_recv.jointAngle),2))-l2*sin(left_frt_recv.jointAngle);
	leg_move_update->y  = (leg_move_update->yL + leg_move_update->yR)/2;
	
		
	//calculate accel of y-axis
	//����y����ļ��ٶ�
	if(accel_flag == 1)
	{
		leg_move_update->y_accel = * (leg_move_update->accel_fliter + INS_ACCEL_Z_ADDRESS_OFFSET) - 9.6437994f;
		accel_flag = 0;
	}
	
	//����
//	if(leg_move_update->y_accel < 0.02f && leg_move_update->y_accel > -0.02f)
//	{
//		leg_move_update->y_accel = 0;
//	}
	
	//���������Ȳ���ĩ��ִ����ת�����ؽڵ�����ص�ϵ��
	leg_move_update->vyR = -(l1-l2*cos(right_frt_recv.jointAngle))*l2*sin(right_frt_recv.jointAngle)/sqrt(pow(l3,2)-pow(l1-l2*cos(right_frt_recv.jointAngle),2))-l2*cos(right_frt_recv.jointAngle);
	leg_move_update->vyL = -(l1-l2*cos(left_frt_recv.jointAngle))*l2*sin(left_frt_recv.jointAngle)/sqrt(pow(l3,2)-pow(l1-l2*cos(left_frt_recv.jointAngle),2))-l2*cos(left_frt_recv.jointAngle);
   
	//����y������ٶ�
	leg_move_update->vy  =  leg_move_update->vy + leg_move_update->y_accel  * JOINTMOTOR_CONTROL_TIME_S ;
	
	if(fabs(leg_move_update->vy - leg_move_update->last_vy) < 0.001f)
	{
		leg_move_update->vy = 0;
	}
	
	leg_move_update->last_vy = leg_move_update->vy;
	
  //calculate chassis euler angle
  //���������̬�Ƕ�
	leg_move_update->chassis_yaw = rad_format(*(leg_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));  
  leg_move_update->chassis_pitch = rad_format(*(leg_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET)) + INS_INIT_PITCH_OFFSET; 
  leg_move_update->chassis_roll = *(leg_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET)+ INS_INIT_ROLL_OFFSET;
	
	//calculate chassis euler angular velocity
	//���������̬���ٶ�
	leg_move_update->chassis_yaw_speed = *(leg_move_update->chassis_INS_angle_speed + INS_YAW_ADDRESS_OFFSET);
	leg_move_update->chassis_pitch_speed =  *(leg_move_update->chassis_INS_angle_speed + INS_ROLL_ADDRESS_OFFSET);
	leg_move_update->chassis_roll_speed = *(leg_move_update->chassis_INS_angle_speed + INS_PITCH_ADDRESS_OFFSET);
}
 
/**
  * @brief          set leg-movement control target-point
  * @param[out]     leg_set: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          �����Ȳ��˶�����Ŀ��ֵ
  * @param[out]     leg_set:"leg_move"����ָ��.
  * @retval         none
  */
static void leg_set_control(leg_move_t *leg_set)
{
	//���ң����S1���ش����е������Ȳ��˶���ҡ�˿���
	if (switch_is_mid(leg_set->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
		leg_set->y_set = LEG_Y_RC_SEN * leg_set->chassis_RC->rc.ch[LEG_Y_CHANNEL] + 0.134;
		leg_set->roll_set = LEG_ROLL_RC_SEN * leg_set->chassis_RC->rc.ch[LEG_ROLL_CHANNEL];
  }
	
	//���ң����S1�����ϵ������Ȳ��������м�λ��
	if (switch_is_up(leg_set->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
		leg_set->y_set = LEY_Y_MID_HEIGHT;
		leg_set->roll_set = 0;
  }
}

/**
  * @brief          control loop, according to control set-point, calculate torque to control motor
  * @param[out]     leg_move_control_loop: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������������Ƶ��
  * @param[out]     leg_move_control_loop:"leg_move"����ָ��.
  * @retval         none
  */
static void jointmotor_control_loop(leg_move_t *leg_move_control_loop)
{
	//����y�����ROLL�����趨ֵ�뷴��ֵ��ƫ��
	leg_move_control_loop->delta_y = leg_move_control_loop->y_set - leg_move_control_loop->y;
	leg_move_control_loop->delta_roll = leg_move_control_loop->roll_set - leg_move_control_loop->chassis_roll;
	
	if(switch_is_mid(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		//�������Ⱥ����ȵ�ĩ��ִ����
		leg_move_control_loop->FL = (KP_Y * leg_move_control_loop->delta_y - KD_Y * leg_move_control_loop->vy + G)/2 + (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;
		leg_move_control_loop->FR = (KP_Y * leg_move_control_loop->delta_y - KD_Y * leg_move_control_loop->vy + G)/2 - (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;	
	}
	
	//����ÿ���ؽڵ���Ŀ�������
	uint8_t i;
	if (switch_is_mid(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]) || switch_is_up(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
	  left_byd_send.T  = -0.5*leg_move_control_loop->FL*leg_move_control_loop->vyL;
	  left_frt_send.T  = 0.5*leg_move_control_loop->FL*leg_move_control_loop->vyL;
	  right_byd_send.T = 0.5*leg_move_control_loop->FR*leg_move_control_loop->vyR;
	  right_frt_send.T = -0.5*leg_move_control_loop->FR*leg_move_control_loop->vyR;
//                                              
	
  }
	//���DOWN�������������Ϊ0
  else if (switch_is_down(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
	  left_byd_send.T = 0;
	  left_frt_send.T = 0;
	  right_byd_send.T = 0;
	  right_frt_send.T = 0;
  }
}