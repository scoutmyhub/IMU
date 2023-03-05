#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"


//电机编码器规整
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/*云台init*/
static void gimbal_init(gimbal_control_t *init);


/*云台状态设置*/
static void gimbal_set_mode(gimbal_control_t *set_mode);

/*云台数据更新*/
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/*云台状态切换*/
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/*计算电机相对中值的相对角度*/
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/*云台控制*/
static void gimbal_set_control(gimbal_control_t *set_control);

/*计算*/
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_GYRO mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_absolute_YAW_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_absolute_PITCH_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_relative_PITCH_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_relative_YAW_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */

static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/*调试函数声明*/
#if GIMBAL_TEST_MODE
static void J_scope_gimbal_test(void);
#endif

/*云台总数据*/
gimbal_control_t gimbal_control;

//motor current 
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    gimbal_init(&gimbal_control);
    //shoot init
    shoot_init();
    //wait for all motor online
    while (1)
    {
        gimbal_set_mode(&gimbal_control);                    
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        shoot_can_set_current = shoot_control_loop();
#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif


//                CAN_cmd_gimbal(0, 0, 0, 0);
                CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, shoot_can_set_current, 0);

/*调试*/
#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @author         RM
  * @param[in]      yaw 中值
  * @param[in]      pitch 中值
  * @param[in]      yaw 最大相对角度
  * @param[in]      yaw 最小相对角度
  * @param[in]      pitch 最大相对角度
  * @param[in]      pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


/**
  * @brief          云台校准计算，将校准记录的最大 最小值 来计算云台 中值和最大最小机械角度
  * @author         RM
  * @param[in]      yaw 中值 指针
  * @param[in]      pitch 中值 指针
  * @param[in]      yaw 最大相对角度 指针
  * @param[in]      yaw 最小相对角度 指针
  * @param[in]      pitch 最大相对角度 指针
  * @param[in]      pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */

/*calibrate task 中调用，用来手动校准P、Y轴*/

bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle      = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle      = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}



static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

/*反装*/
#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
    
    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}


/*云台电机的数据*/
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}


/*云台初始化*/
static void gimbal_init(gimbal_control_t *init)
{
    /*PID DATA*/
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};

    /*电机指针获取*/
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    
    /*陀螺仪数据*/
	
	init->INSData = IMUData_Update();
    init->gimbal_INT_angle_point = init->INSData->Angle;
    init->gimbal_INT_gyro_point = init->INSData->Gyro;
    /*遥控器数据*/
    init->gimbal_rc_ctrl = get_remote_control_point();

    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    gimbal_total_pid_clear(init);

    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;


    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;


}

/*设置云台模式*/
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}


/*云台多项数据的更新*/
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{

    if (feedback_update == NULL)
    {
        return;
    }
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}



static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}



static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_YAW_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_relative_YAW_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_absolute_PITCH_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_relative_PITCH_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
	
	
}


static void gimbal_absolute_PITCH_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    // bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    // if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    // {
    //     if (add > 0.0f)
    //     {
    //         add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //     }
    // }
    // else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    // {
    //     if (add < 0.0f)
    //     {
    //         add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //     }
    // }



    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);

    if (gimbal_motor->absolute_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->absolute_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->absolute_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->absolute_angle_set = gimbal_motor->min_relative_angle;
    }


}

static void gimbal_absolute_YAW_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    // bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    // if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    // {
    //     if (add > 0.0f)
    //     {
    //         add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //     }
    // }
    // else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    // {
    //     if (add < 0.0f)
    //     {
    //         add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //     }
    // }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}


// static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
// {
//     if (gimbal_motor == NULL)
//     {
//         return;
//     }
//     gimbal_motor->relative_angle_set += add;
//     if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
//     {
//         gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//     }
//     else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
//     {
//         gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//     }
// }

static void gimbal_relative_YAW_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
	
    gimbal_motor->relative_angle_set += add;
	
    //gimbal_motor->relative_angle_set += add;
//    //是否超过最大 最小值
//    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
//    {
//        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//    }
//    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
//    {
//        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//    }
}
static void gimbal_relative_PITCH_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}


static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
}



static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }


    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif


static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
