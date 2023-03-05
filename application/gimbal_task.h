

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "IMUTask.h"

//pitch speed close-loop PID params, max out and max iout
#define PITCH_SPEED_PID_KP        7500.0f
#define PITCH_SPEED_PID_KI        50.0f
#define PITCH_SPEED_PID_KD        50.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  5000.0f

//yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP        7000.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        600.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  0.0f

//pitch gyro angle close-loop PID params, max out and max iout
#define PITCH_GYRO_ABSOLUTE_PID_KP 25.0f//25
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
#define YAW_GYRO_ABSOLUTE_PID_KP        28.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   18.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f



#define GIMBAL_TASK_INIT_TIME 201

#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//turn 180��
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
#define TURN_SPEED    0.04f

#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband

#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000006f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0


#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f

#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f


#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5


#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000


#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, 
    GIMBAL_MOTOR_GYRO,   
    GIMBAL_MOTOR_ENCONDE,
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
	const INS_t *INSData;
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
	
} gimbal_control_t;

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
#endif
