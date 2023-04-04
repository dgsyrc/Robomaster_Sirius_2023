/**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       jointmotor_task.c/h
  * @brief      joint motor control task,
  *             关节电机控制任务
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

////腿部运动数据
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
  * @brief          关节电机控制任务，间隔 JOINTMOTOR_CONTROL_TIME_MS 5ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void jointmotor_task(void const *argument)
{
	//wait a time 
	//空闲一段时间
	vTaskDelay(JOINTMOTOR_TASK_INIT_TIME);
	jointmotor_init(&leg_move);
	
	right_byd_send.T = -0;
	left_byd_send.T = 0;
	while(1)
	{
		//joint motor data update
		//关节电机数据更新		
		jointmotor_feedback_update(&leg_move);
		//set joint motor control set-point 
		//关节电机控制量设置
		leg_set_control(&leg_move);		
		//joint motor control value calculate
		//关节电机控制量计算
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
  * @brief          初始化"leg_move"变量，包括遥控器指针初始化，陀螺仪角度和角速度指针初始化，HT03关节电机指针初始化，加速度计滤波数据指针初始化
  * @param[out]     leg_move_init:"leg_move"变量指针.
  * @retval         none
  */
void jointmotor_init(leg_move_t *leg_move_init)
{
	if(leg_move_init == NULL )
	{
		return ;
	}
	
  //get remote control point
  //获取遥控器指针
	leg_move_init->chassis_RC = get_remote_control_point();
	
	//get gyro sensor euler angle point
  //获取陀螺仪姿态角指针
	leg_move_init->chassis_INS_angle = get_INS_angle_point();
	
	//get gyro sensor euler angular velocity
	//获取陀螺仪角速度指针
  leg_move_init->chassis_INS_angle_speed = get_gyro_data_point();
	//get accel fliter data
	//获取加速度计滤波后的数据
	leg_move_init->accel_fliter = get_accel_fliter_data_point();
	
	//initialize all joint motors ID
	//初始化所有关节电机ID
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
	
	//将机体y方向的速度初始化为0
	leg_move_init->vy  = 0;
	//update data
	//更新一下数据
  jointmotor_feedback_update(leg_move_init);
}

/**
  * @brief          joint motor some measure data updata, such as motor angle, robot euler angle and angular velocity，leg height，velocity of y-axis
  * @param[out]     leg_move_update: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          关节电机测量数据更新，包括电机角度，机器人欧拉角度和角速度，腿部高度，y方向速度
  * @param[out]     leg_move_update:"leg_move"变量指针.
  * @retval         none
  */
static void jointmotor_feedback_update(leg_move_t *leg_move_update)
{
	if(leg_move_update == NULL)
	{
		return;
	}

	//关节电机角度值更新
	right_frt_recv.jointAngle = 2.617 + 0.42 - right_frt_recv.Pos;
	left_frt_recv.jointAngle = 2.617 - 0.282 + left_frt_recv.Pos;

	//左右腿部高度以及车体高度更新
	leg_move_update->yR = sqrt(pow(l3,2)-pow(l1-l2*cos(right_frt_recv.jointAngle),2))-l2*sin(right_frt_recv.jointAngle);
	leg_move_update->yL = sqrt(pow(l3,2)-pow(l1-l2*cos(left_frt_recv.jointAngle),2))-l2*sin(left_frt_recv.jointAngle);
	leg_move_update->y  = (leg_move_update->yL + leg_move_update->yR)/2;
	
		
	//calculate accel of y-axis
	//计算y方向的加速度
	if(accel_flag == 1)
	{
		leg_move_update->y_accel = * (leg_move_update->accel_fliter + INS_ACCEL_Z_ADDRESS_OFFSET) - 9.6437994f;
		accel_flag = 0;
	}
	
	//死区
//	if(leg_move_update->y_accel < 0.02f && leg_move_update->y_accel > -0.02f)
//	{
//		leg_move_update->y_accel = 0;
//	}
	
	//计算左右腿部的末端执行力转换到关节点击力矩的系数
	leg_move_update->vyR = -(l1-l2*cos(right_frt_recv.jointAngle))*l2*sin(right_frt_recv.jointAngle)/sqrt(pow(l3,2)-pow(l1-l2*cos(right_frt_recv.jointAngle),2))-l2*cos(right_frt_recv.jointAngle);
	leg_move_update->vyL = -(l1-l2*cos(left_frt_recv.jointAngle))*l2*sin(left_frt_recv.jointAngle)/sqrt(pow(l3,2)-pow(l1-l2*cos(left_frt_recv.jointAngle),2))-l2*cos(left_frt_recv.jointAngle);
   
	//计算y方向的速度
	leg_move_update->vy  =  leg_move_update->vy + leg_move_update->y_accel  * JOINTMOTOR_CONTROL_TIME_S ;
	
	if(fabs(leg_move_update->vy - leg_move_update->last_vy) < 0.001f)
	{
		leg_move_update->vy = 0;
	}
	
	leg_move_update->last_vy = leg_move_update->vy;
	
  //calculate chassis euler angle
  //计算底盘姿态角度
	leg_move_update->chassis_yaw = rad_format(*(leg_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));  
  leg_move_update->chassis_pitch = rad_format(*(leg_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET)) + INS_INIT_PITCH_OFFSET; 
  leg_move_update->chassis_roll = *(leg_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET)+ INS_INIT_ROLL_OFFSET;
	
	//calculate chassis euler angular velocity
	//计算底盘姿态角速度
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
  * @brief          设置腿部运动控制目标值
  * @param[out]     leg_set:"leg_move"变量指针.
  * @retval         none
  */
static void leg_set_control(leg_move_t *leg_set)
{
	//如果遥控器S1开关处于中档，则腿部运动由摇杆控制
	if (switch_is_mid(leg_set->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
		leg_set->y_set = LEG_Y_RC_SEN * leg_set->chassis_RC->rc.ch[LEG_Y_CHANNEL] + 0.134;
		leg_set->roll_set = LEG_ROLL_RC_SEN * leg_set->chassis_RC->rc.ch[LEG_ROLL_CHANNEL];
  }
	
	//如果遥控器S1处于上档，则腿部保持在中间位置
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
  * @brief          控制循环，根据控制设定值，计算输出力矩来控制电机
  * @param[out]     leg_move_control_loop:"leg_move"变量指针.
  * @retval         none
  */
static void jointmotor_control_loop(leg_move_t *leg_move_control_loop)
{
	//计算y方向和ROLL方向设定值与反馈值的偏差
	leg_move_control_loop->delta_y = leg_move_control_loop->y_set - leg_move_control_loop->y;
	leg_move_control_loop->delta_roll = leg_move_control_loop->roll_set - leg_move_control_loop->chassis_roll;
	
	if(switch_is_mid(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		//计算左腿和右腿的末端执行力
		leg_move_control_loop->FL = (KP_Y * leg_move_control_loop->delta_y - KD_Y * leg_move_control_loop->vy + G)/2 + (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;
		leg_move_control_loop->FR = (KP_Y * leg_move_control_loop->delta_y - KD_Y * leg_move_control_loop->vy + G)/2 - (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;	
	}
	
	//计算每个关节电机的控制力矩
	uint8_t i;
	if (switch_is_mid(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]) || switch_is_up(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
	  left_byd_send.T  = -0.5*leg_move_control_loop->FL*leg_move_control_loop->vyL;
	  left_frt_send.T  = 0.5*leg_move_control_loop->FL*leg_move_control_loop->vyL;
	  right_byd_send.T = 0.5*leg_move_control_loop->FR*leg_move_control_loop->vyR;
	  right_frt_send.T = -0.5*leg_move_control_loop->FR*leg_move_control_loop->vyR;
//                                              
	
  }
	//如果DOWN掉，则控制力矩为0
  else if (switch_is_down(leg_move_control_loop->chassis_RC->rc.s[CHASSIS_LEG_CHANNEL]))
  {
	  left_byd_send.T = 0;
	  left_frt_send.T = 0;
	  right_byd_send.T = 0;
	  right_frt_send.T = 0;
  }
}