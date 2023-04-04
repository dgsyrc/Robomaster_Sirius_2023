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

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_task.h"

/*������Ϊģʽ*/
typedef enum
{
  CHASSIS_ZERO_FORCE,  //��������ģʽ
  CHASSIS_BALANCE,     //����ƽ��ģʽ
  CHASSIS_REMOTE,      //����ң��ģʽ
} chassis_behaviour_e;

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
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

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
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif