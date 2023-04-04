#ifndef __PY_LK_MOTOR_H
#define __PY_LK_MOTOR_H

#include "main.h"

//typedef struct
//{
//	int8_t temperature;						//单位度
//	uint16_t Voltage_100mv;				//单位0.1v
//	float Voltage_v;							//由单位0.1v转换
//	uint8_t errorCode;						//bit 0低压保护，bit 3过温保护
//	int16_t iq;										//转矩电流	-2048到2048映射到-33A到33A
//	int16_t speed;								//转速 单位dps
//	uint16_t encoder;							//编码器位置 范围 0-16383
//	
//	uint32_t refreshTime;					//刷新时间
//}LK_Motor_t;

//extern LK_Motor_t LK_Motor[2];

void testGo(int16_t speed);
void readData(uint8_t ID)	;
void readData_Handle(uint8_t* buff);

//------------------------------正式函数-----------------------------------------------------------
#define LK_READ_MOTOR_STATUS_1		0x9A											//12 读取状态1 		反馈温度电压错误字
#define LK_READ_MOTOR_STATUS_2		0x9C											//14 读取状态2 		反馈温度转矩转速编码器位置
#define LK_CLEAN_ERROR_STATUS			0x9B											//13 清除错误代码 	反馈温度电压错误字
#define LK_TURN_OFF_MOTOR					0x80											//16 关闭电机，同时清除之前的状态和控制命令
#define LK_STOP_MOTOR							0x81											//17 停止电机，保留之前的状态和控制命令
#define LK_CONTINUE_RUN_MOTOR			0x82											//18 恢复电机，恢复之前的状态和控制命令

void LK_Motor_SendCtrlMode(uint8_t ID, uint8_t modeCode);		//搭配上面的宏定义用

void LK_Motor_Ctrl_iq(uint8_t ID,int16_t iq);								//20 转矩闭环控制	ID1-2;  iq:-2000到2000 对应-32A到32A
void LK_Motor_Ctrl_speed(uint8_t ID,int32_t speed);					//21 速度闭环控制	ID1-2;  speed对应 0.01dps
//void LK_Motor_Ctrl_pos(uint8_t ID,int32_t totalPos);			//慎用！22 位置闭环控制	totalPos对应总角度 即36000映射到360度

void LK_Motor_DecodeData_Handle(uint8_t MotorID,uint8_t* buff);
void LK_Motor_DebugPrintf_Handle(void);

#endif
