/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             ¼ì²â´íÎóÈÎÎñ£¬ Í¨¹ý½ÓÊÕÊý¾ÝÊ±¼äÀ´ÅÐ¶Ï.Ìá¹© ¼ì²â¹³×Óº¯Êý,´íÎó´æÔÚº¯Êý.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    add a sensor 
    1. in detect_task.h, add the sensor name at the end of errorList,like
    enum errorList
    {
        ...
        XXX_TOE,    //new sensor
        ERROR_LIST_LENGHT,
    };
    2.in detect_init function, add the offlineTime, onlinetime, priority params,like
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3. if XXX_TOE has data_is_error_fun ,solve_lost_fun,solve_data_error_fun function, 
        please assign to function pointer.
    4. when XXX_TOE sensor data come, add the function detect_hook(XXX_TOE) function.
    Èç¹ûÒªÌí¼ÓÒ»¸öÐÂÉè±¸
    1.µÚÒ»²½ÔÚdetect_task.h£¬Ìí¼ÓÉè±¸Ãû×ÖÔÚerrorListµÄ×îºó£¬Ïñ
    enum errorList
    {
        ...
        XXX_TOE,    //ÐÂÉè±¸
        ERROR_LIST_LENGHT,
    };
    2.ÔÚdetect_initº¯Êý,Ìí¼ÓofflineTime, onlinetime, priority²ÎÊý
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.Èç¹ûÓÐdata_is_error_fun ,solve_lost_fun,solve_data_error_funº¯Êý£¬¸³Öµµ½º¯ÊýÖ¸Õë
    4.ÔÚXXX_TOEÉè±¸Êý¾ÝÀ´µÄÊ±ºò, Ìí¼Óº¯Êýdetect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#include "detect_task.h"
#include "cmsis_os.h"


/**
  * @brief          init error_list, assign  offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
/**
  * @brief          ³õÊ¼»¯error_list,¸³Öµ offline_time, online_time, priority
  * @param[in]      time:ÏµÍ³Ê±¼ä
  * @retval         none
  */
static void detect_init(uint32_t time);




 error_t error_list[ERROR_LIST_LENGHT + 1];


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif


/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ¼ì²âÈÎÎ ñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void detect_task(void const *pvParameters)
{
    static uint32_t system_time;
    system_time = xTaskGetTickCount();
    //init,³õÊ¼»¯
    detect_init(system_time);
    //wait a time.¿ÕÏÐÒ»¶ÎÊ±¼ä
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        static uint8_t error_num_display = 0;
        system_time = xTaskGetTickCount();

        error_num_display = ERROR_LIST_LENGHT;
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            //disable, continue
            //Î´Ê¹ÄÜ£¬Ìø¹ý
            if (error_list[i].enable == 0)
            {
                continue;
            }

            //judge offline.ÅÐ¶ÏµôÏß
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
                if (error_list[i].error_exist == 0)
                {
                    //record error and time
                    //¼ÇÂ¼´íÎóÒÔ¼°µôÏßÊ±¼ä
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_time;
                }
                //judge the priority,save the highest priority ,
                //ÅÐ¶Ï´íÎóÓÅÏÈ¼¶£¬ ±£´æÓÅÏÈ¼¶×î¸ßµÄ´íÎóÂë
                if (error_list[i].priority > error_list[error_num_display].priority)
                {
                    error_num_display = i;
                }
                

                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;
                //if solve_lost_fun != NULL, run it
                //Èç¹ûÌá¹©½â¾öº¯Êý£¬ÔËÐÐ½â¾öº¯Êý
                if (error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }
            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                //just online, maybe unstable, only record
                //¸Õ¸ÕÉÏÏß£¬¿ÉÄÜ´æÔÚÊý¾Ý²»ÎÈ¶¨£¬Ö»¼ÇÂ¼²»¶ªÊ§£¬
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].is_lost = 0;
                //ÅÐ¶ÏÊÇ·ñ´æÔÚÊý¾Ý´íÎó
                //judge if exist data error
                if (error_list[i].data_is_error != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }
                //calc frequency
                //¼ÆËãÆµÂÊ
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
/**
  * @brief          »ñÈ¡Éè±¸¶ÔÓ¦µÄ´íÎó×´Ì¬
  * @param[in]      toe:Éè±¸Ä¿Â¼
  * @retval         true(´íÎó) »òÕßfalse(Ã»´íÎó)
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          ¼ÇÂ¼Ê±¼ä
  * @param[in]      toe:Éè±¸Ä¿Â¼
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    
    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }
    
    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the point of error_list
  */
/**
  * @brief          µÃµ½´íÎóÁÐ±í
  * @param[in]      none
  * @retval         error_listµÄÖ¸Õë
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

extern void OLED_com_reset(void);
static void detect_init(uint32_t time)
{
    //ÉèÖÃÀëÏßÊ±¼ä£¬ÉÏÏßÎÈ¶¨¹¤×÷Ê±¼ä£¬ÓÅÏÈ¼¶ offlineTime onlinetime priority
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            {30, 40, 15},   //SBUS
            {10, 10, 11},   //motor1
            {10, 10, 10},   //motor2
            {10, 10, 9},    //motor3
            {10, 10, 8},    //motor4
            {2, 3, 14},     //yaw
            {2, 3, 13},     //pitch
            {10, 10, 12},   //trigger
            {2, 3, 7},      //board gyro
            {5, 5, 7},      //board accel
            {40, 200, 7},   //board mag
            {100, 100, 5},  //referee
            {10, 10, 7},    //rm imu
            {100, 100, 1},  //oled
        };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }

    error_list[OLED_TOE].data_is_error_fun = NULL;
    error_list[OLED_TOE].solve_lost_fun = OLED_com_reset;
    error_list[OLED_TOE].solve_data_error_fun = NULL;

//    error_list[DBUSTOE].dataIsErrorFun = RC_data_is_error;
//    error_list[DBUSTOE].solveLostFun = slove_RC_lost;
//    error_list[DBUSTOE].solveDataErrorFun = slove_data_error;

}
