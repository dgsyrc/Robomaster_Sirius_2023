#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

//3508 motor data
/*3508电机的数据结构体*/
typedef struct
{
    uint16_t ecd;            //编码器码盘值，范围0~8191
    int16_t speed_rpm;       //电机转速，单位RPM
    int16_t given_current;   //转矩电流
    uint8_t temperate;       //电机温度
    int16_t last_ecd;        //上一次的码盘值
} chassis_motor_measure_t;

//翎控9025电机结构体
typedef struct
{
	int8_t temperature;						//单位度
	uint16_t Voltage_100mv;				//单位0.1v
	float Voltage_v;							//由单位0.1v转换
	uint8_t errorCode;						//bit 0低压保护，bit 3过温保护
	float iq;										//转矩电流	-2048到2048映射到-33A到33A
	int16_t speed;								//转速 单位dps
	uint16_t encoder;							//编码器位置 范围 0-16383
	uint32_t refreshTime;					//刷新时间
	uint16_t 	last_angle;	   //上次的绝对角度值0-16383
	uint16_t 	angle;	     //本次绝对角度0-16383
	int32_t		total_angle;//转过的总角度
	int16_t round_cnt; //转过的圈数·
}LK_Motor_t;


extern LK_Motor_t chassis_motor[2];
extern const LK_Motor_t *get_chassis_motor_measure_point(uint8_t i);

extern void LK_Motor_Ctrl_iq(uint8_t ID,int16_t iq);
extern void LK_Motor_Shutdown(uint8_t ID);
void get_total_angle1(LK_Motor_t *p);
void get_total_angle2(LK_Motor_t *p);
#endif