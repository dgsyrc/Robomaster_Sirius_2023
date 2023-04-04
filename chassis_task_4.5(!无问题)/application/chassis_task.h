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

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//����ʼ�����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 1000

//the channel num of controlling vertial movement 
//����ǰ���˶���ң����ͨ������
#define CHASSIS_X_CHANNEL 3

//the channel num of controlling rotation movement
//����YAW����ת�˶���ң����ͨ������
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode
//ѡ�����״̬�Ŀ���ͨ����
#define CHASSIS_MODE_CHANNEL 0

//the channel of choosing leg-movement mode
//ѡ���Ȳ��˶�״̬�Ŀ���ͨ����
#define CHASSIS_LEG_CHANNEL 1

//rocker value (max 660) change to vertial speed (m/s) 
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.0075f

//rocker value change to rotation target angle
//ң������yawң�ˣ�max 660��ת���ɳ�����תĿ��Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.00003f

//ң��ֵһ�׵�ͨ�˲�ʱ���˲�����ֵ
#define CHASSIS_ACCEL_X_NUM 0.1666666667f

//rocker value deadline
//ҡ��������ֵ
#define CHASSIS_RC_DEADLINE 10

//radius of wheel
//���ְ뾶
#define WHEEL_RADIUS 0.07425

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//chassis 3508  motor max control current
//����9025������can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 2000.0f

//chassi forward, back key
//����ǰ���˶����̿��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S

//reducation of 3508 motor
//m3508����ļ��ٱ�
#define M3508_MOTOR_REDUCATION 19.2032f

//m3508 rpm change to chassis speed
//m3508ת��ת��(rpm)ת���ɵ����ٶ�(m/s)�ı�����c=pi*r/(30*k)��kΪ������ٱ�
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00040490766f

//��ص�� ��/s -> ����m/s
// /360*2pi * 0.0675 
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00178097245f

////m3508 rpm change to motor angular velocity
////m3508ת��ת��(rpm)ת��Ϊ�������ٶ�(rad/s)�ı���
//#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.0054533f

//��ص�� ��/s -> rad/s
// /360*2pi
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.01745329252f


//m3508 current change to motor torque
//m3508ת�ص���(-16384~16384)תΪ�ɵ�����ת��(N.m)�ı���
//c=20/16384*0.3��0.3Ϊת�س���(N.m/A)
//#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//��ص�� ��/s -> rad/s
// Ť�س���0.81f (N.m/A)
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.81f



//single chassis motor max torque
//�������̵���������
#define MAX_WHEEL_TORQUE 5.8f

//chassis forward or back max speed
//�����˶��������ǰ���˶��ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f

//LQR feedback parameter
//LQR��������ϵ��
//����
#define LQR_K1 -2.2361f
#define LQR_K2 -3.7258f
#define LQR_K3 -18.7288f 
#define LQR_K4 -3.2695f
//#define LQR_K15 2.2361f
#define LQR_K15 2.2361f
#define LQR_K16 0.2015f
#define LQR_K25 -LQR_K15
#define LQR_K26 -LQR_K16

//#define LQR_K1 -2.5730f
//#define LQR_K2 -4.2867f
//#define LQR_K3 -19.1921f
//#define LQR_K4 -3.3808f
//#define LQR_K15 2.2361f
//#define LQR_K16 0.5015f
//#define LQR_K25 -LQR_K15
//#define LQR_K26 -LQR_K16
//���̿���ģʽ
typedef enum
{
  CHASSIS_REMOTE_MODE,  //ң��ģʽ
  CHASSIS_BALANCE_MODE, //ƽ��ģʽ
  CHASSIS_DOWN_MODE,    //DOWNģʽ
} chassis_mode_e;

//���̵�����ݽṹ��
typedef struct
{
  const LK_Motor_t *chassis_motor_measure;
  fp32 speed;           //�������λ���ٶ�
	fp32 omg;             //����������ת�ٶ�
	fp32 torque;          //����������
	fp32 torque_set;      //�����������趨ֵ
  int16_t give_current; //������Ƶ����趨ֵ
} chassis_motor_t;

//�����˶����ݽṹ��
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //��ȡң����ָ��
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  const fp32 *chassis_INS_angle_speed;       //��ȡ�����ǽ��������ת���ٶ�ָ��
	chassis_mode_e chassis_mode;               //���̿���ģʽ״̬��
  chassis_mode_e last_chassis_mode;          //�����ϴο���ģʽ״̬��
  chassis_motor_t motor_chassis[2];          //���̵������

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
	fp32 distance;                    //������ǰλ��
  fp32 vx;                          //�����ٶȣ�ǰ��Ϊ������Ϊ������λm/s
  fp32 omg;                         //���̺ϳ��ֵĽ��ٶȣ���λrad/s
  fp32 vx_set;                      //�����ٶ��趨ֵ��ǰ��Ϊ������Ϊ������λm/s
  fp32 chassis_yaw_set;             //����yaw��Ƕ��趨ֵ
  fp32 delta_angle;                 //����yaw��Ƕ��趨ֵ��yaw��Ƕȵ�ǰֵ֮��

  fp32 vx_max_speed;                //ǰ����������ٶȣ���λm/s
  fp32 vx_min_speed;                //���˷�������ٶȣ���λm/s
  fp32 chassis_yaw;                 //���������Ƿ����ĵ�ǰyaw�Ƕ�
  fp32 chassis_pitch;               //���������Ƿ����ĵ�ǰpitch�Ƕ�
  fp32 chassis_roll;                //���������Ƿ����ĵ�ǰroll�Ƕ�
  fp32 chassis_yaw_speed;           //���������Ƿ����ĵ�ǰyaw���ٶ�
	fp32 chassis_pitch_speed;         //���������Ƿ����ĵ�ǰpitch���ٶ�
	fp32 chassis_roll_speed;          //���������Ƿ����ĵ�ǰroll���ٶ�
} chassis_move_t;

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
extern void chassis_task(void const *pvParameters);

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
extern void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector);

#endif