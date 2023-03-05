#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"



//??否启用底??
#define CHASSIS_DISABLE 0

//in the beginning of task ,wait a time
#define CHASSIS_TASK_INIT_TIME 357

/*前后遥控器的通道号码*/
#define CHASSIS_X_CHANNEL 1
/*水平速度的通道号码*/
#define CHASSIS_Y_CHANNEL 0
/*在一些模式下，用遥控器控制转向*/
#define CHASSIS_WZ_CHANNEL 2
/*底盘开??*/
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f 

//rocker value deadline
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

/*不理??*/
#define MOTOR_DISTANCE_TO_CENTER 0.157f

/*底盘控制任务间隔*/
#define CHASSIS_CONTROL_TIME_MS 2
/*一阶低通滤波间隔的时间*/
#define CHASSIS_CONTROL_TIME 0.002f
/*下面加速度的跟新用??*/
/*chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;*/
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大发送电流??
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//W,A,S,D
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

/*TOP*/
#define CHASSIS_TOP_KEY KEY_PRESSED_OFFSET_F

//m3508 rmp change to chassis speed
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

////single chassis motor max speed
//#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

/*麦轮解算用到的参数*//*不理解*/
#define CHASSIS_WZ_SET_SCALE 0.1f

//底盘不动时，最大摆动角度
#define SWING_NO_MOVE_ANGLE 0.7f
//底盘运动,最大摆动角度
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.001f
#define M3505_MOTOR_SPEED_PID_KD 2.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 25.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 7.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 14000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f

/*底盘参数*/
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f
#define Fast_Run_flag KEY_PRESSED_OFFSET_SHIFT   //冲刺
#define CLASSIS_CHONGNEN_MODE KEY_PRESSED_OFFSET_Q
 //底盘运动过程最大前进速度   40
#define MAX_WHEEL_SPEED_40 0.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_40 0.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_40_t 1.5f
//底盘运动过程最大平移速度   40
#define MAX_WHEEL_SPEED_40_t 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_40 0.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_40_t 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_Z_40 10.0f

//底盘运动过程最大前进速度   45
#define MAX_WHEEL_SPEED_45 0.9f
#define NORMAL_MAX_CHASSIS_SPEED_X_45 0.9f
#define NORMAL_MAX_CHASSIS_SPEED_X_45_t 1.7f
//底盘运动过程最大平移速度   45
#define MAX_WHEEL_SPEED_45_t 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_45 0.9f
#define NORMAL_MAX_CHASSIS_SPEED_Y_45_t 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_Z_45 10.0f

//底盘运动过程最大前进速度   50
#define MAX_WHEEL_SPEED_50 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_X_50 1.1f
#define NORMAL_MAX_CHASSIS_SPEED_X_50_t 1.5f
//底盘运动过程最大平移速度   50
#define MAX_WHEEL_SPEED_50_t 1.6f
#define NORMAL_MAX_CHASSIS_SPEED_Y_50 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_Y_50_t 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_Z_50 10.0f

//底盘运动过程最大前进速度   55
#define MAX_WHEEL_SPEED_55 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_X_55 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_X_55_t 2.0f
//底盘运动过程最大平移速度   55
#define MAX_WHEEL_SPEED_55_t 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y_55 1.3f
#define NORMAL_MAX_CHASSIS_SPEED_Y_55_t 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_Z_55 10.0f

//底盘运动过程最大前进速度  60
#define MAX_WHEEL_SPEED_60 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_X_60 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_X_60_t 1.8f
//底盘运动过程最大平移速度  60
#define MAX_WHEEL_SPEED_60_t 1.9f
#define NORMAL_MAX_CHASSIS_SPEED_Y_60 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_Y_60_t 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Z_60 20.0f

//底盘运动过程最大前进速度   80
#define MAX_WHEEL_SPEED_80 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_80 1.6f
#define NORMAL_MAX_CHASSIS_SPEED_X_80_t 2.0f
//底盘运动过程最大平移速度   80
#define MAX_WHEEL_SPEED_80_t 2.1f
#define NORMAL_MAX_CHASSIS_SPEED_Y_80 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_Y_80_t 1.6f
#define NORMAL_MAX_CHASSIS_SPEED_Z_80 20.0f
//底盘运动过程最大平移速度   80飞坡
#define MAX_WHEEL_SPEED_80_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_80_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_80_FLY 4.7f



//底盘运动过程最大前进速度   100
#define MAX_WHEEL_SPEED_100 1.8f
#define NORMAL_MAX_CHASSIS_SPEED_X_100 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_100_t 2.2f
//底盘运动过程最大平移速度  100
#define MAX_WHEEL_SPEED_100_t 2.3f
#define NORMAL_MAX_CHASSIS_SPEED_Y_100 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_100_t 1.9f
#define NORMAL_MAX_CHASSIS_SPEED_Z_100 20.0f
//底盘运动过程最大平移速度   100飞坡
#define MAX_WHEEL_SPEED_100_FLY 3.5f
#define NORMAL_MAX_CHASSIS_SPEED_X_100_FLY 3.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_100_FLY 3.5f

//底盘运动过程最大前进速度   120
#define MAX_WHEEL_SPEED_120 3.4f
#define NORMAL_MAX_CHASSIS_SPEED_X_120 3.4f
#define NORMAL_MAX_CHASSIS_SPEED_X_120_t 3.4f
//底盘运动过程最大平移速度  120
#define MAX_WHEEL_SPEED_120_t 3.3f
#define NORMAL_MAX_CHASSIS_SPEED_Y_120 3.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y_120_t 3.0f
#define NORMAL_MAX_CHASSIS_SPEED_Z_120 10.0f
//底盘运动过程最大平移速度   120飞坡
#define MAX_WHEEL_SPEED_120_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_120_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_120_FLY 4.7f


typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       
  CHASSIS_VECTOR_RAW,            
  CHASSIS_VECTOR_TOP,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;


typedef struct 
{
//	fp32 POWER_LIMIT;        // 80.0f
//	fp32 WARNING_POWER;    // 40.0f
//	fp32 WARNING_POWER_BUFF;  //50.0f
//	fp32  MAX_WHEEL_SPEED;   // 1.5f;
	fp32 i;
}chassis_power_parameter;

typedef struct
{
  float Limit_k;
  float Real_PowerBuffer;
  float Max_PowerBuffer;
  float CHAS_TotalOutput;
  float CHAS_LimitOutput;
}CHASSIS_PowerLimit_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;             
  const gimbal_motor_t *chassis_yaw_motor;  
  const gimbal_motor_t *chassis_pitch_motor; 
  const fp32 *chassis_INS_angle;
  chassis_power_parameter *power_parameter;
  chassis_mode_e chassis_mode;          
  chassis_mode_e last_chassis_mode;      
  chassis_motor_t motor_chassis[4];         
  pid_type_def motor_speed_pid[4];        
  pid_type_def chassis_angle_pid;      

  first_order_filter_type_t chassis_cmd_slow_set_vx; 
  first_order_filter_type_t chassis_cmd_slow_set_vy;  

  fp32 vx;                         
  fp32 vy;                  
  fp32 wz;                      
  fp32 vx_set;                 
  fp32 vy_set;                 
  fp32 wz_set;                 
  fp32 chassis_relative_angle;     
  fp32 chassis_relative_angle_set; 
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  
  fp32 vx_min_speed;  
  fp32 vy_max_speed; 
  fp32 vy_min_speed; 
  fp32 vz_max_speed;  //左右方向最大速度 单位m/s
  fp32 vz_min_speed;  //左右方向最小速度 单位m/s
  fp32 chassis_yaw;  
  fp32 chassis_pitch; 
  fp32 chassis_roll; 

  ramp_function_source_t classis_ramp;
  CHASSIS_PowerLimit_t *CHASSIS_PowerLimit;

} chassis_move_t;




static void chassis_jude_mode(chassis_move_t *chassis_move_update);
void chassis_power_control(chassis_move_t *chassis_power_control);
float TOP_Priority(void);
/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval          none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
