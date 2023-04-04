#ifndef __PY_LK_MOTOR_H
#define __PY_LK_MOTOR_H

#include "main.h"

//typedef struct
//{
//	int8_t temperature;						//��λ��
//	uint16_t Voltage_100mv;				//��λ0.1v
//	float Voltage_v;							//�ɵ�λ0.1vת��
//	uint8_t errorCode;						//bit 0��ѹ������bit 3���±���
//	int16_t iq;										//ת�ص���	-2048��2048ӳ�䵽-33A��33A
//	int16_t speed;								//ת�� ��λdps
//	uint16_t encoder;							//������λ�� ��Χ 0-16383
//	
//	uint32_t refreshTime;					//ˢ��ʱ��
//}LK_Motor_t;

//extern LK_Motor_t LK_Motor[2];

void testGo(int16_t speed);
void readData(uint8_t ID)	;
void readData_Handle(uint8_t* buff);

//------------------------------��ʽ����-----------------------------------------------------------
#define LK_READ_MOTOR_STATUS_1		0x9A											//12 ��ȡ״̬1 		�����¶ȵ�ѹ������
#define LK_READ_MOTOR_STATUS_2		0x9C											//14 ��ȡ״̬2 		�����¶�ת��ת�ٱ�����λ��
#define LK_CLEAN_ERROR_STATUS			0x9B											//13 ���������� 	�����¶ȵ�ѹ������
#define LK_TURN_OFF_MOTOR					0x80											//16 �رյ����ͬʱ���֮ǰ��״̬�Ϳ�������
#define LK_STOP_MOTOR							0x81											//17 ֹͣ���������֮ǰ��״̬�Ϳ�������
#define LK_CONTINUE_RUN_MOTOR			0x82											//18 �ָ�������ָ�֮ǰ��״̬�Ϳ�������

void LK_Motor_SendCtrlMode(uint8_t ID, uint8_t modeCode);		//��������ĺ궨����

void LK_Motor_Ctrl_iq(uint8_t ID,int16_t iq);								//20 ת�رջ�����	ID1-2;  iq:-2000��2000 ��Ӧ-32A��32A
void LK_Motor_Ctrl_speed(uint8_t ID,int32_t speed);					//21 �ٶȱջ�����	ID1-2;  speed��Ӧ 0.01dps
//void LK_Motor_Ctrl_pos(uint8_t ID,int32_t totalPos);			//���ã�22 λ�ñջ�����	totalPos��Ӧ�ܽǶ� ��36000ӳ�䵽360��

void LK_Motor_DecodeData_Handle(uint8_t MotorID,uint8_t* buff);
void LK_Motor_DebugPrintf_Handle(void);

#endif
