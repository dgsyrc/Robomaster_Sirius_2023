#include "motor_control.h"
#include "usart.h"


#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 
 
 //发送数据时，去掉CRC后有30个字节，7个完整的uint32_t型，所以len=7
 //接收数据时，74个字节，len=18
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}


//发送的数据整合，比如力矩*256给电机
int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0]=0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->K_P,  0.0f,   15.999f);
		SATURATE(motor_s->K_W,  0.0f,   31.999f);
		SATURATE(motor_s->T,   -127.99f,  127.99f);
		SATURATE(motor_s->W,   -256.00f,  256.00f);
		SATURATE(motor_s->Pos, -823549.0f,  823549.0f);
	//这里数据拼接还没写完
	motor_s->motor_send_data.head.motorID = motor_s->id;
	motor_s->motor_send_data.cmd.mode = motor_s->mode;
	motor_s->motor_send_data.cmd.Pos = motor_s->Pos/6.2832f*16384;
	motor_s->motor_send_data.cmd.T = (motor_s->T*256);
	motor_s->motor_send_data.cmd.W = (motor_s->W*128);
    motor_s->motor_send_data.cmd.K_P = (motor_s->K_P*2048);
	motor_s->motor_send_data.cmd.K_W = (motor_s->K_W*1024);
	motor_s->motor_send_data.CRC32 = crc32_core((uint32_t *)&motor_s->motor_send_data,7);
	motor_s->motor_send_data.cmd.ModifyBit = 0xFF;
	
//	motor_s->motor_send_data.mode.id   = motor_s->id;
//    motor_s->motor_send_data.mode.status  = motor_s->mode;
//    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
//    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
//    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
//    motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
//    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
//    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}


int extract_data(MOTOR_recv *motor_r)
{
	
	if(motor_r->motor_recv_data.CRCdata != crc32_core((uint32_t*)&motor_r->motor_recv_data,18))
	{       
		motor_r->correct = 0;
        return motor_r->correct;
	}

//    if(motor_r->motor_recv_data.CRC16 !=
//        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
//        // printf("[WARNING] Receive data CRC error");
//        motor_r->correct = 0;
//        return motor_r->correct;
//	}
	
	
	//一样没写数据拼接
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.head.motorID;
        motor_r->mode = motor_r->motor_recv_data.cmd.mode;
        motor_r->Temp = motor_r->motor_recv_data.cmd.Temp;
        motor_r->MError = motor_r->motor_recv_data.cmd.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.cmd.W/128);
        motor_r->T = ((float)motor_r->motor_recv_data.cmd.T/256);
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.cmd.Pos) / 16384 /9.1;

        return motor_r->correct;
    }
}

HAL_StatusTypeDef right_joint_motor_control(MOTOR_send *pData, MOTOR_recv *rData)
{
    uint16_t rxlen = 0;

    modify_data(pData);

    HAL_UART_Transmit(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 
		

    HAL_UARTEx_ReceiveToIdle(&huart6, (uint8_t *)rData, sizeof(rData->motor_recv_data), &rxlen, 10);
		

    if(rxlen == 0)

      return HAL_TIMEOUT;

    if(rxlen != sizeof(rData->motor_recv_data))
			return HAL_ERROR;

    uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
    if(rp[0] == 0xFE && rp[1] == 0xEE)
    {
        rData->correct = 1;
        extract_data(rData);
        return HAL_OK;
    }
    
    return HAL_ERROR;
}


//左侧电机
HAL_StatusTypeDef left_joint_motor_control(MOTOR_send *pData, MOTOR_recv *rData)
{
    uint16_t rxlen = 0;

    modify_data(pData);

    HAL_UART_Transmit(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 
		

    HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rData, sizeof(rData->motor_recv_data), &rxlen, 10);
		

    if(rxlen == 0)

      return HAL_TIMEOUT;

    if(rxlen != sizeof(rData->motor_recv_data))
			return HAL_ERROR;

    uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
    if(rp[0] == 0xFE && rp[1] == 0xEE)
    {
        rData->correct = 1;
        extract_data(rData);
        return HAL_OK;
    }
    
    return HAL_ERROR;
}



