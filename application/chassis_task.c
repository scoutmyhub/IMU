#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "referee_lib.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @param[out]     chassis_move_init: "chassis_move" valiable point
 * @retval         none
 */

static void chassis_init(chassis_move_t *chassis_move_init);

/**
 * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
 * @param[out]     chassis_move_mode: "chassis_move" valiable point
 * @retval         none
 */

static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
 * @param[out]     chassis_move_transit: "chassis_move" valiable point
 * @retval         none
 */

void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
 * @brief          chassis some measure data updata, such as motor speed, euler angleï¿½ï¿½ robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */

static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 *
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

chassis_move_t chassis_move;

fp32 POWER_LIMIT = 0.0f;
fp32 WARNING_POWER = 0.0f;
fp32 WARNING_POWER_BUFF = 50.0f;
fp32 climb;
fp32 climb_1;
fp32 scopperil = 0;
int16_t max_power=4500;//å®šä¹‰æœ€å¤§åŠŸçŽ‡å€?
fp32  MAX_WHEEL_SPEED=1.5f;
/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
uint8_t k = 0;
void chassis_task(void const *pvParameters)
{
  // wait a time
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  // chassis init
  chassis_init(&chassis_move);
  // make sure all chassis motor is online,
  //  while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
  //  {
  //    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  //  }

  while (1)
  {
    // set chassis control mode
    chassis_set_mode(&chassis_move);
    // when mode changes, some data save
    chassis_mode_change_control_transit(&chassis_move);
    // chassis data update
    chassis_feedback_update(&chassis_move);
    // set chassis control set-point
    chassis_set_contorl(&chassis_move);
    // chassis control pid calculate
    chassis_control_loop(&chassis_move);

    // make sure  one motor is online at least, so that the control CAN message can be received
    //    if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
    //    {
    // when remote control is offline, chassis motor should receive zero current.
    if (CHASSIS_DISABLE)
    {
      CAN_cmd_chassis(0, 0, 0, 0);
    }
    else
    {
      CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                      chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
    }
    //    }
    // os delay
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
    chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @param[out]     chassis_move_init: "chassis_move" valiable point
 * @retval         none
 */

static void chassis_init(chassis_move_t *chassis_move_init)
{
  if (chassis_move_init == NULL)
  {
    return;
  }

  // chassis motor speed PID
  const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

  // chassis angle PID
  const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;

  chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
  // get remote control point
  chassis_move_init->chassis_RC = get_remote_control_point();
  // get gyro sensor euler angle point
//  chassis_move_init->chassis_INS_angle = get_INS_angle_point();
  // get gimbal motor data point
  chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
  chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

  // get chassis motor data point,  initialize motor speed PID
  for (i = 0; i < 4; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
  }
  // initialize angle PID
  PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

  // first order low-pass filter  replace ramp function
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
  ramp_init(&chassis_move_init->classis_ramp, CHASSIS_CONTROL_TIME , 2.5f, 0.0f);
//  // max and min speed
//  chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
//  chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

//  chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
//  chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  // update data
  chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
 * @param[out]     chassis_move_mode: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }
  // in file "chassis_behaviour.c"
  chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
 * @param[out]     chassis_move_transit: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
  if (chassis_move_transit == NULL)
  {
    return;
  }

  if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
  {
    return;
  }

  // change to follow gimbal angle mode
  if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
  {
    chassis_move_transit->chassis_relative_angle_set = 0.0f;
  }
  // change to follow chassis yaw angle
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
  {
    chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }
  // change to no follow angle
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
  {
    chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }
  else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_TOP) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_TOP)
  {
    chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
  }

  chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
 * @brief          chassis some measure data updata, such as motor speed, euler angleï¿½ï¿½ robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
  if (chassis_move_update == NULL)
  {
    return;
  }

  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    // update motor speed, accel is differential of speed PID
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  // calculate vertical speed, horizontal speed ,rotation speed, left hand rule
  chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

  // calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
  chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
  chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
  //chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
  
  chassis_jude_mode(chassis_move_update);
}
/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
 * @retval         none
 */

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;

  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  /*W A S D*/
  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
  }

  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
  
}
/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
  if (chassis_move_control == NULL)
  {
    return;
  }

  fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

  if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
  {

    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

    sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
    chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

    chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);

    chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
    chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->vz_min_speed, chassis_move_control->vz_max_speed);
    chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);

    ramp_calc_min(&chassis_move_control->classis_ramp, 3.0f);
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
  {
    fp32 delat_angle = 0.0f;
    chassis_move_control->chassis_yaw_set = rad_format(angle_set);
    delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
    chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
    chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	ramp_calc_min(&chassis_move_control->classis_ramp, 3.0f);
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
  {
    //"angle_set" is rotation speed set-point
    chassis_move_control->wz_set = angle_set;
    chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	ramp_calc_min(&chassis_move_control->classis_ramp, 3.0f);
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
  {
    chassis_move_control->vx_set = vx_set;
    chassis_move_control->vy_set = vy_set;
    chassis_move_control->wz_set = angle_set;
    chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
	ramp_calc_min(&chassis_move_control->classis_ramp, 3.0f);
  }

  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_TOP)
  {
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

    sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);

    chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
    chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

    chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);

    chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_yaw_motor->relative_angle);

    chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    ramp_calc(&chassis_move_control->classis_ramp, 2.0f);
  }
}

/**
 * @brief          four mecanum wheels speed is calculated by three param.
 * @param[in]      vx_set: vertial speed
 * @param[in]      vy_set: horizontal speed
 * @param[in]      wz_set: rotation speed
 * @param[out]     wheel_speed: four mecanum wheels speed
 * @retval         none
 */

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{

  //  wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  //  wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  //  wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  //  wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[0] = -vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

  // mecanum wheel speed calculation
  chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                        chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
  /*¿ª»·Ö±½ÓÊä³ö*/
  // if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
  // {

  //   for (i = 0; i < 4; i++)
  //   {
  //     chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
  //   }
  //   return;
  // }

  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
    temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
  }

  // calculate pid
  for (i = 0; i < 4; i++)
  {
    PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set - chassis_move_control_loop->classis_ramp.out);
    chassis_move_control_loop->classis_ramp.max_value = scopperil;
//    PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
  }

  chassis_power_control(chassis_move_control_loop);

  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
  }
}
	fp32 accel_speed = 0.0f;
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;

	
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //scale down
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                if(chassis_power < POWER_LIMIT)
                {
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                    
                }
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    } 

    
    total_current = 0.0f;
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
	accel_speed = (60.0f - chassis_power_buffer)/60.0f;
    if(accel_speed <= 1)
		accel_speed = 1;

    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;//*pow(accel_speed, 2);
        chassis_power_control->motor_speed_pid[1].out*=current_scale;//*pow(accel_speed, 2);
        chassis_power_control->motor_speed_pid[2].out*=current_scale*2.0f;//*pow(accel_speed, 2);
        chassis_power_control->motor_speed_pid[3].out*=current_scale*2.0f;//*pow(accel_speed, 2);
		
    }
}

float chassis_power = 0.0;
static void chassis_jude_mode(chassis_move_t *chassis_move_update)
{
  if (Chassis_Power_Limit() == 40)
  {
    POWER_LIMIT = 40.0f;
    WARNING_POWER = 30.0f;

    max_power = 4000;
    scopperil = 1.4f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_40;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_40;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_40;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_40;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_40;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_40;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_40;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_40_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_40_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_40_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_40_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_40_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_40;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_40;
      }
    }
  }

  else if (Chassis_Power_Limit() == 45)
  {
    POWER_LIMIT = 45.0f;
    WARNING_POWER = 35.0f;

    max_power = 4500;
    scopperil = 1.6f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_45;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_45;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_45;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_45;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_45;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_45;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_45;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_45_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_45_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_45_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_45_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_45_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_45;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_45;
      }
    }
  }

  else if (Chassis_Power_Limit() == 50)
  {
    POWER_LIMIT = 50.0f;
    WARNING_POWER = 40.0f;

    max_power = 5000;
    scopperil = 1.8f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_50;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_50;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_50;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_50;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_50;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_50;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_50;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_50_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_50_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_50_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_50_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_50_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_50;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_50;
      }
    }
  }

  else if (Chassis_Power_Limit() == 55)
  {
    POWER_LIMIT = 55.0f;
    WARNING_POWER = 45.0f;

    max_power = 5500;
    scopperil = 1.9f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_55;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_55;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_55;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_55;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_55;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_55;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_55;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_55_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_55_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_55_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_55_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_55_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_55;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_55;
      }
    }
  }
  else if (Chassis_Power_Limit() == 60)
  {
    POWER_LIMIT = 60.0f;
    WARNING_POWER = 50.0f;

    max_power = 6000;
    scopperil = 2.2f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_60;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_60 - TOP_Priority();
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_60 - TOP_Priority();

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_60 - TOP_Priority();
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_60 - TOP_Priority();

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_60;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_60;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_60_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_60_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_60_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_60_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_60_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_60;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_60;
      }
    }
  }
  //    	if(JudgeeverydataE.game_robot_status_t.robot_level==2)
  else if (Chassis_Power_Limit() == 80)
  {
    POWER_LIMIT = 80.0f;
    WARNING_POWER = 70.0f;

    max_power = 8000;
    scopperil = 2.5f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_80;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_80;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_80;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_80;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_80;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_80;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_80;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_80_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_80_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_80_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_80_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_80_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_80;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_80;
      }
    }
    //				  if(chassis_move_update->chassis_RC->key.v & Fast_Run_flag)
    //					  {
    //							   WARNING_POWER_BUFF  =240.0f;
    //							 	 MAX_WHEEL_SPEED=	MAX_WHEEL_SPEED_80_FLY;
    //						     chassis_move_update->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X_80_FLY;
    //						     chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_80_FLY;
    //
    //				    		 chassis_move_update->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y_80_FLY;
    //				    		 chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_80_FLY;
    //
    //								 chassis_move_update->vz_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Z_80;
    //							   chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_80;
    //						}
  }

  // 	  if(JudgeeverydataE.game_robot_status_t.robot_level==3)
  else if (Chassis_Power_Limit() == 100)
  {
    POWER_LIMIT = 100.0f;
    WARNING_POWER = 90.0f;

    max_power = 10000;
    scopperil = 3.0f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_100;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_100;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_100;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_100;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_100;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_100;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_100;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {

        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_100_t;
        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_100_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_100_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_100_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_100_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_100;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_100;
      }
      if (chassis_move_update->chassis_RC->key.v & Fast_Run_flag)
      {
        WARNING_POWER_BUFF = 240.0f;
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_100_FLY;
        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_100_FLY;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_100_FLY;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_100_FLY;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_100_FLY;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_100;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_100;
      }
    }
  }

  else if (Chassis_Power_Limit() == 120)
  {
    POWER_LIMIT = 120.0f;
    WARNING_POWER = 110.0f;

    max_power = 10000;
    scopperil = 3.2f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_120;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_120;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_120;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_120;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_120;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_120;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_120;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {

        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_120_t;
        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_120_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_120_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_120_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_120_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_120;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_120;
      }
      if (chassis_move_update->chassis_RC->key.v & Fast_Run_flag)
      {
        WARNING_POWER_BUFF = 240.0f;
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_120_FLY;
        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_120_FLY;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_120_FLY;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_120_FLY;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_120_FLY;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_120;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_120;
      }
    }
  }
  else
  {
    POWER_LIMIT = 40.0f;
    WARNING_POWER = 30.0f;

    max_power = 4000;
    scopperil = 1.4f;
    if (chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE)
    {
      MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_40;
      chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_40;
      chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_40;

      chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_40;
      chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_40;

      chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_40;
      chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_40;
    }
    else if (!(chassis_move_update->chassis_RC->key.v & CLASSIS_CHONGNEN_MODE))
    {
      if (!(chassis_move_update->chassis_RC->key.v & Fast_Run_flag))
      {
        MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_40_t;

        chassis_move_update->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X_40_t;
        chassis_move_update->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X_40_t;

        chassis_move_update->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y_40_t;
        chassis_move_update->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y_40_t;

        chassis_move_update->vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z_40;
        chassis_move_update->vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z_40;
      }
    }
  }
}


float TOP_Priority(void)
{
  if(chassis_move.chassis_mode == CHASSIS_VECTOR_TOP)
  {
	return 0.2;
  }
  else
    return 0;
}








