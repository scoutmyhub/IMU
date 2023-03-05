/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//射击发射开关通道数据
#define Shoot_RC_Channel_right 0
#define Shoot_RC_Channel_left 1

//云台模式使用的开关通道
#define SHOOT_CONTROL_TIME GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE 100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_OPEN_FIRE KEY_PRESSED_OFFSET_R

//(组合键)升高或降低摩擦轮的转速
#define FIRE_UP_DONE_RPM KEY_PRESSED_OFFSET_CTRL
#define FIRE_UP_RPM KEY_PRESSED_OFFSET_Q
#define FIRE_DONE_RPM KEY_PRESSED_OFFSET_E

//降低 / 升高摩擦轮的转速
#define RPM_UP_DATA 5.0f
#define RPM_DONE_DATA -5.0f

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 15
//鼠标长按判断
#define PRESS_LONG_TIME 400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清弹
#define RC_LONG_TIME 2000
/*复位*/
#define SHOOT_OFF_honming KEY_PRESSED_OFFSET_X
/*摩擦轮半径  单位：m*/
#define FIRE_RADIUS 0.03f

//电机rmp 变化成 旋转速度的比例----来自官方
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f

//拨弹速度
#define TRIGGER_SPEED 10.0f
#define CONTINUE_TRIGGER_SPEED 15.0f
#define READY_TRIGGER_SPEED 5.0f


//卡单时间 以及反转时间
#define BLOCK_TIME 10
#define REVERSE_TIME 2
#define REVERSE_SPEED_LIMIT 13.0f
#define  BLOCK_TRIGGER_SPEED  1.0f

/*摩擦轮电机最大输出限制*/
#define FIRE_left_SPEED_PID_MAX_OUT 20000.0f
#define FIRE_left_SPEED_PID_MAX_IOUT 0.0f

#define FIRE_right_SPEED_PID_MAX_OUT 20000.0f
#define FIRE_right_SPEED_PID_MAX_IOUT 0.0f
//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 0.3f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 0.9f

#define TRIGGER_SPEED_PID_KP 9.0f
#define TRIGGER_SPEED_PID_KI 0.0f
#define TRIGGER_SPEED_PID_KD 1.0f
/*摩擦轮电机PID*/
#define Fire_left_SPEED_PID_KP 13
#define Fire_left_SPEED_PID_KI 0.18
#define Fire_left_SPEED_PID_KD 0

#define Fire_right_SPEED_PID_KP 13
#define Fire_right_SPEED_PID_KI 0.18
#define Fire_right_SPEED_PID_KD 0

/*拨弹盘双环PID输出限制*/
#define BULLET_ANG_PID_MAX_OUT 10000.0f
#define BULLET_ANG_PID_MAX_IOUT 0.0f
#define BULLET_SPEED_PID_MAX_OUT 10000.0f
#define BUFFET_SPEED_PID_MAX_IOUT 1000.0f

#define SHOOT_HEAT_REMAIN_VALUE 80

typedef enum
{
  SHOOT_STOP = 0,
  SHOOT_READY,
  SHOOT_BULLET,
  SHOOT_CONTINUE_BULLET,
  SHOOT_DONE,
} shoot_mode_e;

/*摩擦轮电机数据*/
typedef struct
{
  float Fire_right_speed;
  float Fire_left_speed;
  int16_t Fire_left_speed_set;
  int16_t Fire_right_speed_set;
  int16_t Given_current;  
}Fire_Data_t;

/*按键记录*/
typedef struct
{
  int8_t LAST_SHOOT_SWITCH_CLIP_KEY;//记录弹仓按键
  int8_t LAST_SHOOT_SWITCH_KEYBOARD;//记录射击按键
  int8_t LAST_SWTCH_RIGHT_SET;//记录右RC
  int8_t LAST_SWTCH_LEFT_SET;//记录左RC值
  int8_t LAST_KEY_SET;
  bool_t LAST_PRESS_L;
  bool_t LAST_PRESS_R;
}Control_SET_t;

typedef struct
{
  bool_t SHOOT_SWITCH_KEYBOARD;/*卡蛋*/
  bool_t SHOOT_OPEN_CLIP_FLAG;/*弹舱*/
  bool_t ShootFlag;
}Control_FALG_t;


/*按键时间的记录*/
typedef struct
{
  uint8_t KEY;
  uint32_t KEY_TIME;
  uint32_t switch_time;/*用于赛前清弹*/
  uint32_t shoot_count_cnt;
  uint32_t Manual_Reset_FLAG;/*复位*/
}Control_Time_t;

/*滤波*/
typedef struct 
{
  ramp_function_source_t fric1_ramp;
  ramp_function_source_t fric2_ramp;
}Ramp_Data_t;

/*卡弹*/
typedef struct
{
  int block_time;
  int Reset_ECD;
}Turn_Back_Data_t;


typedef struct
{
  const motor_measure_t *shoot_motor_measure;
  const RC_ctrl_t *RC_ctrl;
  Control_SET_t Last_Control_SET;
  Control_FALG_t Control_FALG;
  Fire_Data_t Fire_Data;
  Control_Time_t Control_Time;
  Ramp_Data_t Ramp_Data_t;
  Turn_Back_Data_t Turn_Back_Data;
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
// float Get_fire_rate(uint16_t speed);
float Get_SHOOT_RPM(void);
#endif
