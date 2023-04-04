/**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.1.0     Apr-01-2022     HYX             1. done,apply to two-wheeled Robot
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"


#define rc_deadband_limit(input, output, dealine)    \
{                                                    \
	if ((input) > (dealine) || (input) < -(dealine))   \
	{                                                  \
		(output) = (input);                              \
	}                                                  \
	else                                               \
	{                                                  \
		(output) = 0;                                    \
	}                                                  \
}

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//�����˶�����
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
	
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);
	

    while (1)
    {
		//���õ��̿��Ʒ�ʽ
		chassis_set_mode(&chassis_move);
		//�������ݸ���
        chassis_feedback_update(&chassis_move);
		//���̿���������
		chassis_set_contorl(&chassis_move);	
		//���̿���LQR����
        chassis_control_loop(&chassis_move);
		        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE)))
        {
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE))
            {
                //CAN_cmd_chassis(0,0);
				LK_Motor_Shutdown(1);
				LK_Motor_Shutdown(2);
            }
            else
            {
							//send control current
							//���Ϳ��Ƶ���
							LK_Motor_Ctrl_iq(1,chassis_move.motor_chassis[0].give_current);
							LK_Motor_Ctrl_iq(2,chassis_move.motor_chassis[1].give_current);
            }
        }
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          "chassis_move" valiable initialization, include first order low-pass filter initialization, remote control data point initialization, 
  *                 3508 chassis motors data point initialization, gyro sensor angle point and angular velocity point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"chassis_move"����������һ�׵�ͨ�˲�����ʼ����ң����ָ���ʼ����3508���̵��ָ���ʼ���������ǽǶȺͽ��ٶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

		//parameter of first order low-pass
		//һ�׵�ͨ�˲������˲�����
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
		
    //in beginning�� chassis mode is DOWN
    //���̿���״̬ΪDOWNģʽ
    chassis_move_init->chassis_mode = CHASSIS_DOWN_MODE;
		
    //get remote control point
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
		
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
		
		//get gyro sensor euler angular velocity
		//��ȡ�����ǽ��ٶ�ָ��
		chassis_move_init->chassis_INS_angle_speed = get_gyro_data_point();

    //get chassis motor data point
    //��ȡ���̵������ָ��
		uint8_t i;
    for (i = 0; i < 2; i++)
    {
			chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    }
		
    //first order low-pass filter  replace ramp function,and init parameters
    //��һ���˲�����б���������ɣ���ʼ����ز���
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);

    //max and min speed
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          chassis some measure data updata, such as motor angular velocity, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£����������ת�ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 2; i++)
    {
			//update motor speed
			//���µ������λ���ٶ�
			chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed;
			
			//update motor angular velocity
			//���µ���������ٶ�
			chassis_move_update->motor_chassis[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed;
			
			//update motor torque
			//���µ��ת��
			chassis_move_update->motor_chassis[i].torque = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->iq;
    }

    //calculate chassis euler angle
    //���������̬�Ƕ�
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET)) + INS_INIT_PITCH_OFFSET;
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) + INS_INIT_ROLL_OFFSET;
		
		//calculate chassis euler angular velocity
		//���������̬���ٶ�
		chassis_move_update->chassis_yaw_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_PITCH_ADDRESS_OFFSET);
		chassis_move_update->chassis_pitch_speed =  *(chassis_move_update->chassis_INS_angle_speed +INS_YAW_ADDRESS_OFFSET );
		chassis_move_update->chassis_roll_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_ROLL_ADDRESS_OFFSET);
	  
		//calculate chassis velocity and synthesis angular velocity
		//��������ٶȺͺϳ��ֽ��ٶ�
	  chassis_move_update->vx = ((chassis_move_update->motor_chassis[0].speed) - (chassis_move_update->motor_chassis[1].speed))/2 ;
	  chassis_move_update->omg = ((chassis_move_update->motor_chassis[0].omg) - (chassis_move_update->motor_chassis[1].omg))/2 ;
		
			//update distance
			//���µ�ǰλ��ֵ
		chassis_move_update->distance = ((chassis_move_update->motor_chassis[0].chassis_motor_measure->round_cnt*0.424115+chassis_move_update->motor_chassis[0].chassis_motor_measure->angle*0.424115/65536)+(chassis_move_update->motor_chassis[1].chassis_motor_measure->round_cnt*0.424115+chassis_move_update->motor_chassis[1].chassis_motor_measure->angle*0.424115/65536))/2;
}

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ң����ͨ��ֵ�����������ٶ��趨ֵ
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL )
    {
        return;
    }
    
    int16_t vx_channel ;
    fp32 vx_set_channel ;
		
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		
    //��ң�˲���ת��Ϊ�˶�����
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
		
    //keyboard set speed set-point
    //���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    
		//stop command, need not slow change, set zero derectly
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
}

/**
  * @brief          set chassis control target-point, movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���Ŀ��ֵ, �˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
      return;
    }

    fp32 vx_set = 0.0f, angle_set = 0.0f;
		
    //get movement control target-points, ��ȡ�˶�����Ŀ��ֵ
    chassis_behaviour_control_set(&vx_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_REMOTE_MODE)
    {
			chassis_move_control->delta_angle = 0.0f;
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_BALANCE_MODE)
    {
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_DOWN_MODE)
		{
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
      chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
}

/**
  * @brief          control loop, according to control set-point, calculate current to control motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_torque = 0.0f, torque_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;

		/*�������������=SUM[LQR����ϵ��*(״̬����Ŀ��ֵ-״̬��������ֵ)]*/
		/*ע��������������ص������ţ��Լ���״̬��������ֵ��������*/
	  //�����������
	  chassis_move_control_loop->motor_chassis[0].torque_set = - (LQR_K1*(chassis_move_control_loop->distance)+LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) - LQR_K3*chassis_move_control_loop->chassis_pitch  + LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) + LQR_K15*chassis_move_control_loop->delta_angle + LQR_K16*chassis_move_control_loop->chassis_yaw_speed);
	  //�����������
	  chassis_move_control_loop->motor_chassis[1].torque_set =   (LQR_K1*(chassis_move_control_loop->distance)+LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) - LQR_K3*chassis_move_control_loop->chassis_pitch  + LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) + LQR_K25*chassis_move_control_loop->delta_angle + LQR_K26*chassis_move_control_loop->chassis_yaw_speed);

		/*�����DOWN������Ƶ���ֵ����Ϊ0*/
	  if (chassis_move_control_loop->chassis_mode == CHASSIS_DOWN_MODE) 
    {
			for (i = 0; i < 2; i++)
			{
				chassis_move_control_loop->motor_chassis[i].give_current = 0;
		  }
			return;
    }
		
		/*�������򻬣��򻬻�ʹ�������������ת��ֱ����������ֵ����ʱ������ֵ����Ϊ0*/
		if(fabs(chassis_move_control_loop->motor_chassis[0].omg)>=30 || fabs(chassis_move_control_loop->motor_chassis[1].omg)>=30)
		{      
			for(i=0;i<2;i++)
			{
				chassis_move_control_loop->motor_chassis[i].give_current = 0;
			}
			return ;
		}
		
		//calculate the max torque in two wheels, limit the max torque
    //�����������ת�أ������������ת��
    for (i = 0; i < 2; i++)
    {
			temp = fabs(chassis_move_control_loop->motor_chassis[i].torque_set);
			if (max_torque < temp)
			{
				max_torque = temp;
			}
    }

    if (max_torque > MAX_WHEEL_TORQUE)
    {
			torque_rate = MAX_WHEEL_TORQUE / max_torque;
			for (i = 0; i < 2; i++)
			{
				chassis_move_control_loop->motor_chassis[i].torque_set *= torque_rate;
			}
    }

    //��ֵ����ֵ
    for (i = 0; i < 2; i++)
    {
			chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_chassis[i].torque_set / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);
			chassis_move_control_loop->motor_chassis[i].give_current *= 62.5f;
	}
}
