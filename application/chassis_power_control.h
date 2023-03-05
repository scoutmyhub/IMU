///**
//  ****************************(C) COPYRIGHT 2019 DJI****************************
//  * @file       chassis_power_control.c/h
//  * @brief      chassis power control.底盘功率控制
//  * @note       this is only controling 80 w power, mainly limit motor current set.
//  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
//  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
//  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
//  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
//  *             (包括max_vx_speed, min_vx_speed)
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

////底盘3508最大can发送电流值
//#define MAX_MOTOR_CAN_CURRENT 16000.0f
//#define Fast_Run_flag KEY_PRESSED_OFFSET_SHIFT   //冲刺
//#define CLASSIS_CHONGNEN_MODE KEY_PRESSED_OFFSET_CTRL    //减速
////底盘运动过程最大前进速度   40
//#define MAX_WHEEL_SPEED_40 0.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_40 0.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_40_t 1.5f
////底盘运动过程最大平移速度   40
//#define MAX_WHEEL_SPEED_40_t 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_40 0.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_40_t 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_40 10.0f

////底盘运动过程最大前进速度   45
//#define MAX_WHEEL_SPEED_45 0.9f
//#define NORMAL_MAX_CHASSIS_SPEED_X_45 0.9f
//#define NORMAL_MAX_CHASSIS_SPEED_X_45_t 1.7f
////底盘运动过程最大平移速度   45
//#define MAX_WHEEL_SPEED_45_t 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_45 0.9f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_45_t 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_45 10.0f

////底盘运动过程最大前进速度   50
//#define MAX_WHEEL_SPEED_50 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_50 1.1f
//#define NORMAL_MAX_CHASSIS_SPEED_X_50_t 1.5f
////底盘运动过程最大平移速度   50
//#define MAX_WHEEL_SPEED_50_t 1.6f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_50 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_50_t 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_50 10.0f

////底盘运动过程最大前进速度   55
//#define MAX_WHEEL_SPEED_55 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_55 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_55_t 2.0f
////底盘运动过程最大平移速度   55
//#define MAX_WHEEL_SPEED_55_t 2.0f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_55 1.3f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_55_t 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_55 10.0f

////底盘运动过程最大前进速度  60
//#define MAX_WHEEL_SPEED_60 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_X_60 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_X_60_t 1.8f
////底盘运动过程最大平移速度  60
//#define MAX_WHEEL_SPEED_60_t 1.9f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_60 1.2f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_60_t 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_60 20.0f

////底盘运动过程最大前进速度   80
//#define MAX_WHEEL_SPEED_80 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_80 1.6f
//#define NORMAL_MAX_CHASSIS_SPEED_X_80_t 2.0f
////底盘运动过程最大平移速度   80
//#define MAX_WHEEL_SPEED_80_t 2.1f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_80 1.4f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_80_t 1.6f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_80 20.0f
////底盘运动过程最大平移速度   80飞坡
//#define MAX_WHEEL_SPEED_80_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_80_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_80_FLY 4.7f



////底盘运动过程最大前进速度   100
//#define MAX_WHEEL_SPEED_100 1.8f
//#define NORMAL_MAX_CHASSIS_SPEED_X_100 1.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_100_t 2.2f
////底盘运动过程最大平移速度  100
//#define MAX_WHEEL_SPEED_100_t 2.3f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_100 1.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_100_t 1.9f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_100 20.0f
////底盘运动过程最大平移速度   100飞坡
//#define MAX_WHEEL_SPEED_100_FLY 3.5f
//#define NORMAL_MAX_CHASSIS_SPEED_X_100_FLY 3.5f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_100_FLY 3.5f

////底盘运动过程最大前进速度   120
//#define MAX_WHEEL_SPEED_120 3.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_120 3.2f
//#define NORMAL_MAX_CHASSIS_SPEED_X_120_t 3.0f
////底盘运动过程最大平移速度  120
//#define MAX_WHEEL_SPEED_120_t 3.1f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_120 2.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_120_t 2.8f
//#define NORMAL_MAX_CHASSIS_SPEED_Z_120 10.0f
////底盘运动过程最大平移速度   120飞坡
//#define MAX_WHEEL_SPEED_120_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_X_120_FLY 4.7f
//#define NORMAL_MAX_CHASSIS_SPEED_Y_120_FLY 4.7f

///**
//  * @brief          limit the power, mainly limit motor current
//  * @param[in]      chassis_power_control: chassis data 
//  * @retval         none
//  */
///**
//  * @brief          限制功率，主要限制电机电流
//  * @param[in]      chassis_power_control: 底盘数据
//  * @retval         none
//  */
//extern void chassis_power_control(chassis_move_t *chassis_power_control);

#endif
