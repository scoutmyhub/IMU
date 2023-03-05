///**
//  ****************************(C) COPYRIGHT 2019 DJI****************************
//  * @file       chassis_power_control.c/h
//  * @brief      chassis power control.���̹��ʿ���
//  * @note       this is only controling 80 w power, mainly limit motor current set.
//  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
//  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
//  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
//  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
//  *             (����max_vx_speed, min_vx_speed)
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
//  *
//  @verbatim
//  ==============================================================================

//  ==============================================================================
//  @endverbatim
//  ****************************(C) COPYRIGHT 2019 DJI****************************
//  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"

////����3508���can���͵���ֵ
//#define MAX_MOTOR_CAN_CURRENT 16000.0f
//#define Fast_Run_flag KEY_PRESSED_OFFSET_SHIFT   //���
//#define CLASSIS_CHONGNEN_MODE KEY_PRESSED_OFFSET_CTRL    //����
////�����˶��������ǰ���ٶ�   40
//#define MAX_WHEEL_SPEED_40 0.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_40 0.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_40_t 1.5f
////�����˶��������ƽ���ٶ�   40
//#define MAX_WHEEL_SPEED_40_t 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_40 0.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_40_t 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_40 10.0f

////�����˶��������ǰ���ٶ�   45
//#define MAX_WHEEL_SPEED_45 0.9f
//#define NORMAL_MAX_CHASSIS_SPEED_X_45 0.9f
//#define NORMAL_MAX_CHASSIS_SPEED_X_45_t 1.7f
////�����˶��������ƽ���ٶ�   45
//#define MAX_WHEEL_SPEED_45_t 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_45 0.9f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_45_t 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_45 10.0f

////�����˶��������ǰ���ٶ�   50
//#define MAX_WHEEL_SPEED_50 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_50 1.1f
//#define NORMAL_MAX_CHASSIS_SPEED_X_50_t 1.5f
////�����˶��������ƽ���ٶ�   50
//#define MAX_WHEEL_SPEED_50_t 1.6f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_50 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_50_t 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_50 10.0f

////�����˶��������ǰ���ٶ�   55
//#define MAX_WHEEL_SPEED_55 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_55 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_55_t 2.0f
////�����˶��������ƽ���ٶ�   55
//#define MAX_WHEEL_SPEED_55_t 2.0f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_55 1.3f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_55_t 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_55 10.0f

////�����˶��������ǰ���ٶ�  60
//#define MAX_WHEEL_SPEED_60 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_X_60 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_X_60_t 1.8f
////�����˶��������ƽ���ٶ�  60
//#define MAX_WHEEL_SPEED_60_t 1.9f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_60 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_60_t 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_60 20.0f

////�����˶��������ǰ���ٶ�   80
//#define MAX_WHEEL_SPEED_80 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_80 1.6f
//#define NORMAL_MAX_CHASSIS_SPEED_X_80_t 2.0f
////�����˶��������ƽ���ٶ�   80
//#define MAX_WHEEL_SPEED_80_t 2.1f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_80 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_80_t 1.6f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_80 20.0f
////�����˶��������ƽ���ٶ�   80����
//#define MAX_WHEEL_SPEED_80_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_80_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_80_FLY 4.7f



////�����˶��������ǰ���ٶ�   100
//#define MAX_WHEEL_SPEED_100 1.8f
//#define NORMAL_MAX_CHASSIS_SPEED_X_100 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_100_t 2.2f
////�����˶��������ƽ���ٶ�  100
//#define MAX_WHEEL_SPEED_100_t 2.3f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_100 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_100_t 1.9f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_100 20.0f
////�����˶��������ƽ���ٶ�   100����
//#define MAX_WHEEL_SPEED_100_FLY 3.5f
//#define NORMAL_MAX_CHASSIS_SPEED_X_100_FLY 3.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_100_FLY 3.5f

////�����˶��������ǰ���ٶ�   120
//#define MAX_WHEEL_SPEED_120 3.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_120 3.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_120_t 3.0f
////�����˶��������ƽ���ٶ�  120
//#define MAX_WHEEL_SPEED_120_t 3.1f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_120 2.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_120_t 2.8f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_120 10.0f
////�����˶��������ƽ���ٶ�   120����
//#define MAX_WHEEL_SPEED_120_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_120_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_120_FLY 4.7f

///**
//  * @brief          limit the power, mainly limit motor current
//  * @param[in]      chassis_power_control: chassis data 
//  * @retval         none
//  */
///**
//  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
//  * @param[in]      chassis_power_control: ��������
//  * @retval         none
//  */
//extern void chassis_power_control(chassis_move_t *chassis_power_control);

#endif
