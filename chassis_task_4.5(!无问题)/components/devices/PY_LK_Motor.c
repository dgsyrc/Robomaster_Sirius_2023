/*
��ص�������������б�д

2023.�������

*/
#include "PY_LK_Motor.h"
#include "can.h"

#include "stdio.h"

LK_Motor_t LK_Motor[2];

void testGo(int16_t speed)			
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x141;				//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//���ݳ���8λ
	
	TxData[0]	=	0xA1;							//ת�رջ�����
	TxData[4] = speed & 0xFF;
	TxData[5] = speed >> 8;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}

void readData(uint8_t ID)			
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x141;				//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//���ݳ���8λ
	
	TxData[0]	=	0x9A;							//��ȡ״̬
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}
void readData_Handle(uint8_t* buff)
{
	uint8_t readID = buff[0];
	if(readID == 0x9A)
	{
		//printf("OK\n");
		LK_Motor[0].temperature = buff[1];
		LK_Motor[0].Voltage_100mv = (buff[3] << 8) | buff[4];
		LK_Motor[0].errorCode = buff[7];
	}
	else
	{
		printf("err\n");
	}
}
void LK_Motor_DecodeData_Handle(uint8_t MotorID,uint8_t* buff)
{
	uint8_t ctrlCode = buff[0];													//������
	
	if((MotorID != 1) && (MotorID != 2))return;
	
	switch(ctrlCode)
	{
		
		case 0x9A:										//��ȡ���״̬1
		{
			LK_Motor[MotorID-1].temperature = buff[1];
			LK_Motor[MotorID-1].Voltage_100mv = (buff[4] << 8) | buff[3];
			LK_Motor[MotorID-1].errorCode = buff[7];
		}break;
		case 0x9C:										//��ȡ���״̬2
		case 0xA1:										//20 ת�رջ���������
		case 0xA2:										//21 �ٶȱջ���������
		case 0xA3:										//22 λ�ñջ�����1
		case 0xA4:										//23 λ�ñջ�����2
		case 0xA5:										//22 λ�ñջ�����3
		case 0xA6:										//23 λ�ñջ�����4
		{
			LK_Motor[MotorID-1].temperature = buff[1];
			LK_Motor[MotorID-1].iq 			= (buff[3] << 8) | buff[2];
			LK_Motor[MotorID-1].speed 	= (buff[5] << 8) | buff[4];
			LK_Motor[MotorID-1].encoder = (buff[7] << 8) | buff[6];
		}break;
		case 0x9D:break;							//��ȡ���״̬3���������
		
		default:break;
	}
	LK_Motor[MotorID-1].refreshTime = HAL_GetTick();
	if(LK_Motor[MotorID-1].Voltage_100mv != 0)
		LK_Motor[MotorID-1].Voltage_v = LK_Motor[MotorID-1].Voltage_100mv*10.0;
}



//---------------------------���״̬���� ״̬����----------------------------------------//

void LK_Motor_SendCtrlMode(uint8_t ID, uint8_t modeCode)		//����LK_ϵ�к궨����
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	
	TxData[0]	=	modeCode;					//��ȡ״̬
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}

//---------------------------����˶�+״̬����---------------------------------------------//
void LK_Motor_Ctrl_iq(uint8_t ID,int16_t iq)								//20 ת�رջ�����	-2000��2000 ��Ӧ-32A��32A
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//���ݳ���8λ
	
	TxData[0]	=	0xA1;							//ת�رջ�����
	TxData[4] = iq & 0xFF;				//��8
	TxData[5] = iq >> 8;					//��8
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}
void LK_Motor_Ctrl_speed(uint8_t ID,int32_t speed)					//21 �ٶȱջ�����	speed��Ӧ 0.01dps
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//���ݳ���8λ
	
	TxData[0]	=	0xA2;							//21 �ٶȱջ�����
	TxData[4] = (speed >>  0) & 0xFF;
	TxData[5] = (speed >>  8) & 0xFF;
	TxData[6] = (speed >> 16) & 0xFF;
	TxData[7] = (speed >> 24) & 0xFF;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}
//void LK_Motor_Ctrl_pos(uint8_t ID,int32_t totalPos)				//���ã� 22 λ�ñջ�����	totalPos��Ӧ�ܽǶ� ��36000ӳ�䵽360��
//{
//	uint8_t TxData[8] = {0};
//	uint32_t TxMainBox = 0;
//	CAN_TxHeaderTypeDef TxHeader;
//	
//	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
//	TxHeader.IDE =  CAN_ID_STD;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 0x08;					//���ݳ���8λ
//	
//	TxData[0]	=	0xA3;							//22 λ�ñջ�����
//	TxData[4] = (totalPos >>  0) & 0xFF;
//	TxData[5] = (totalPos >>  8) & 0xFF;
//	TxData[6] = (totalPos >> 16) & 0xFF;
//	TxData[7] = (totalPos >> 24) & 0xFF;
//	
//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
//}

//---------------------------printf����ģʽ---------------------------------------------//
int32_t iq = 0;

void LK_Motor_DebugPrintf_Handle(void)
{
	static uint32_t lastPrintfTime;
	
	//readData(1);
	//LK_Motor_SendCtrlMode(2,LK_READ_MOTOR_STATUS_2);
	LK_Motor_Ctrl_speed(2,iq);
	LK_Motor_Ctrl_speed(1,iq);
	if(HAL_GetTick() - lastPrintfTime > 100)			//���ڴ�ӡƵ��10Hz
	{
		lastPrintfTime = HAL_GetTick();
		//printf("Motor[%d]:%d,%d,%d,%d",q,LK_Motor[q].temperature,LK_Motor[q].iq,LK_Motor[q].speed,LK_Motor[q].encoder);
		//printf("vol:%.1f",LK_Motor[q].Voltage_v);
		//printf(",err:%d", LK_Motor[q].errorCode);
//		printf(",tem:%d", LK_Motor[q].temperature);
//		printf(",spd:%d", LK_Motor[q].speed);
//		printf(",enc:%d", LK_Motor[q].encoder);
//		printf(",iq:%d",  LK_Motor[q].iq);
//		printf("\n");
	}
}


