/**
  ****************************(C) COPYRIGHT 2021 SuperPower****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function，CAN1 receives chassis motor and joint motor data,
  *             and CAN send function sends motor parameters to control motor.
  *             这里是CAN中断接收函数，CAN1接收关节电机和底盘电机数据，CAN发送函数发送电机参数控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  v1.2.0     Nov-24-2021     HYX             1. add joint motor
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 SuperPower****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

static LK_Motor_t chassis_motor[2];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


//解包
void LK_Motor_DecodeData_Handle(uint8_t MotorID,uint8_t* buff)
{
	uint8_t ctrlCode = buff[0];													//控制字
	
	if((MotorID != 1) && (MotorID != 2))return;
	
	switch(ctrlCode)
	{
		
		case 0x9A:										//读取电机状态1
		{
			chassis_motor[MotorID-1].temperature = buff[1];
			chassis_motor[MotorID-1].Voltage_100mv = (buff[4] << 8) | buff[3];
			chassis_motor[MotorID-1].errorCode = buff[7];
		}break;
		case 0x9C:										//读取电机状态2
		case 0xA1:										//20 转矩闭环控制命令
		case 0xA2:										//21 速度闭环控制命令
		case 0xA3:										//22 位置闭环控制1
		case 0xA4:										//23 位置闭环控制2
		case 0xA5:										//22 位置闭环控制3
		case 0xA6:										//23 位置闭环控制4
		{
			chassis_motor[MotorID-1].temperature = buff[1];
			chassis_motor[MotorID-1].iq 			= ((int16_t)(buff[3] << 8) | buff[2])/2048.0f*33.0f;
			chassis_motor[MotorID-1].speed 	= (buff[5] << 8) | buff[4];
			chassis_motor[MotorID-1].angle = (buff[7] << 8) | buff[6];
			switch (MotorID){
				case 1:
			get_total_angle1(&chassis_motor[MotorID-1]);
				break;
				case 2:
			get_total_angle2(&chassis_motor[MotorID-1]);
				break;
					default:break;
			}

		}break;
		case 0x9D:break;							//读取电机状态3（相电流）
		
		default:break;
	}
	chassis_motor[MotorID-1].refreshTime = HAL_GetTick();
	if(chassis_motor[MotorID-1].Voltage_100mv != 0)
		chassis_motor[MotorID-1].Voltage_v = chassis_motor[MotorID-1].Voltage_100mv*10.0;
}



/**
  * @brief          hal CAN fifo call back, receive motor data，Unpack the data and store it in the corresponding array
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据,并将数据解包后存入相应数组
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHead; 
    uint8_t Rxdata[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, Rxdata);
	
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, Rxdata);

    switch (RxHead.StdId)
    {		
				//LK电机
				case 0x205:
				case 0x206:
				{
					uint8_t i;
					i = RxHead.StdId - 0x205;										//计算绝对偏移地址				//将获取到的报文信息传入子函数进行处理
					//get_total_angle(&moto_chassis[i]);							//计算绝对角度
				}
				case 0x141:			//新添加，翎控电机ID1
				case 0x142:			//新添加，翎控电机ID2
				{
					//readData_Handle(RxData);
					LK_Motor_DecodeData_Handle(RxHead.StdId - 0x140,Rxdata);
				}
				break;

        default:break;
    }
	}
	
	
}

void testGo(int16_t speed)			
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x141;				//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//数据长度8位
	
	TxData[0]	=	0xA1;							//转矩闭环控制
	TxData[4] = speed & 0xFF;
	TxData[5] = speed >> 8;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}


//LK电机控制
void LK_Motor_Ctrl_iq(uint8_t ID,int16_t iq)								//20 转矩闭环控制	-2000到2000 对应-32A到32A
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//数据长度8位
	
	TxData[0]	=	0xA1;							//转矩闭环控制
	TxData[4] = iq & 0xFF;				//低8
	TxData[5] = iq >> 8;					//高8
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}
void LK_Motor_Ctrl_speed(uint8_t ID,int32_t speed)					//21 速度闭环控制	speed对应 0.01dps
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//数据长度8位
	
	TxData[0]	=	0xA2;							//21 速度闭环控制
	TxData[4] = (speed >>  0) & 0xFF;
	TxData[5] = (speed >>  8) & 0xFF;
	TxData[6] = (speed >> 16) & 0xFF;
	TxData[7] = (speed >> 24) & 0xFF;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}

void LK_Motor_Shutdown(uint8_t ID)
{
	uint8_t TxData[8] = {0};
	uint32_t TxMainBox = 0;
	CAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.StdId = 0x140 + ID;	//ID	0x140+ID
	TxHeader.IDE =  CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;					//数据长度8位
	
	TxData[0]	=	0x80;						

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMainBox);
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         chassis motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         底盘电机数据指针
  */
const LK_Motor_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &chassis_motor[i];
}


/**
*@bref 电机上电角度=0， 之后用这个函数更新9025电机的相对开机后（为0）的相对角度。
	*/
//#define ABS(x)	( (x>0) ? (x) : (-x) )
void get_total_angle1(LK_Motor_t *p){															//放入中断更新角度
	
	if(p->angle - p->last_angle > 32768)										//计算电机旋转了几圈
		p->round_cnt --;
	else if (p->angle - p->last_angle < -32768)						//计算电机旋转了几圈
		p->round_cnt ++;
		p->last_angle = p->angle;
}

void get_total_angle2(LK_Motor_t *p){															//放入中断更新角度
	
	if(p->angle - p->last_angle > 32768)										//计算电机旋转了几圈
		p->round_cnt ++;
	else if (p->angle - p->last_angle < -32768)						//计算电机旋转了几圈
		p->round_cnt --;
		p->last_angle = p->angle;
}