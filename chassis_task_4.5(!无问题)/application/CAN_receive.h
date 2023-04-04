#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

//3508 motor data
/*3508��������ݽṹ��*/
typedef struct
{
    uint16_t ecd;            //����������ֵ����Χ0~8191
    int16_t speed_rpm;       //���ת�٣���λRPM
    int16_t given_current;   //ת�ص���
    uint8_t temperate;       //����¶�
    int16_t last_ecd;        //��һ�ε�����ֵ
} chassis_motor_measure_t;

//���9025����ṹ��
typedef struct
{
	int8_t temperature;						//��λ��
	uint16_t Voltage_100mv;				//��λ0.1v
	float Voltage_v;							//�ɵ�λ0.1vת��
	uint8_t errorCode;						//bit 0��ѹ������bit 3���±���
	float iq;										//ת�ص���	-2048��2048ӳ�䵽-33A��33A
	int16_t speed;								//ת�� ��λdps
	uint16_t encoder;							//������λ�� ��Χ 0-16383
	uint32_t refreshTime;					//ˢ��ʱ��
	uint16_t 	last_angle;	   //�ϴεľ��ԽǶ�ֵ0-16383
	uint16_t 	angle;	     //���ξ��ԽǶ�0-16383
	int32_t		total_angle;//ת�����ܽǶ�
	int16_t round_cnt; //ת����Ȧ����
}LK_Motor_t;


extern LK_Motor_t chassis_motor[2];
extern const LK_Motor_t *get_chassis_motor_measure_point(uint8_t i);

extern void LK_Motor_Ctrl_iq(uint8_t ID,int16_t iq);
extern void LK_Motor_Shutdown(uint8_t ID);
void get_total_angle1(LK_Motor_t *p);
void get_total_angle2(LK_Motor_t *p);
#endif