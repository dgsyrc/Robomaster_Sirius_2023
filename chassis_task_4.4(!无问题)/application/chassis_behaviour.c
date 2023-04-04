  /**
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ����ң������ֵ������������Ϊ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *  v2.1.0     Feb-14-2022     HYX             1. delete some behaviours
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 SuperPower****************************
  */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_down_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/*������Ϊģʽ����*/
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    /*ң�������õ�����Ϊģʽ*/
		
		/*���ң������S2���ش�����λ������ƽ��ģʽ*/
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_BALANCE;
    }
		/*���ң������S2���ش��ڵ�λ����������ģʽ*/
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
		/*���ң������S2���ش��ڸ�λ������ң��ģʽ*/
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_REMOTE;
    }

    /*������Ϊģʽѡ��һ�����̿���ģʽ*/
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_DOWN_MODE; 
    }
    else if (chassis_behaviour_mode == CHASSIS_BALANCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_BALANCE_MODE; 
    }
    else if (chassis_behaviour_mode == CHASSIS_REMOTE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_REMOTE_MODE;
    }
}

/**
  * @brief          set control target-point. according to different control mode, usually call different control function.
  * @param[out]     vx_set, usually controls vertical movement.
  * @param[out]     angle_set, usually controls rotation movement.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          ���ÿ���Ŀ����.���ݲ�ͬ���̿���ģʽ������ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     angle_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_BALANCE)
    {
			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_REMOTE)
    {
			chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          ����DOWN������Ϊ״̬���£�Ŀ���ٶȺ�Ŀ��ת�Ǿ�Ϊ0
  * @param[in]      vx_set ǰ�����ٶ�,��ֵ��ʾǰ���ٶȣ���ֵ��ʾ�����ٶ�
  * @param[in]      angle_set Ŀ��ת��
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_down_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
    *vx_set = 0.0f;
    *angle_set = 0.0f;
}

/**
  * @brief          ����ƽ�����Ϊ״̬���£�Ŀ���ٶȺ�Ŀ��ת�Ǿ�Ϊ0
  * @param[in]      vx_set ǰ�����ٶ�,��ֵ��ʾǰ���ٶȣ���ֵ��ʾ�����ٶ�
  * @param[in]      angle_set Ŀ��ת��
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
    *vx_set = -0.0f;
    *angle_set = 0.0f;
}

/**
  * @brief          ����ң�ص���Ϊ״̬���£�Ŀ���ٶ���ҡ��3ͨ����ӳ�䣬Ŀ��ת����ҡ��2ͨ����ӳ��
  * @param[in]      vx_set ǰ�����ٶ�,��ֵ��ʾǰ���ٶȣ���ֵ��ʾ�����ٶ�
  * @param[in]      angle_set Ŀ��ת�ǣ���Χ-PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, chassis_move_rc_to_vector);
		
    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}