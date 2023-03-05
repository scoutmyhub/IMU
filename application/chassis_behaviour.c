#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);






//highlight, the variable chassis behaviour mode 
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
static CHASSIS_MODE_CONTROL_T* CHASSIS_MODE_CONTROL;


/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    
    if(chassis_move_mode->chassis_RC->key.v & CHASSIS_TOP_KEY && CHASSIS_MODE_CONTROL->LAST_CHASSIS_TOP_KEY & CHASSIS_TOP_KEY)
    {
        CHASSIS_MODE_CONTROL->CHASSIS_TOP_KEY_T = !CHASSIS_MODE_CONTROL->CHASSIS_TOP_KEY_T;
    }
    CHASSIS_MODE_CONTROL->LAST_CHASSIS_TOP_KEY = chassis_move_mode->chassis_RC->key.v;

    //remote control  set chassis behaviour mode
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) /*&& !CHASSIS_MODE_CONTROL->CHASSIS_TOP_KEY_T*/ )
    {
        //can change to CHASSIS_ZERO_FORCE,CHASSIS_NO_MOVE,CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
        //CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,CHASSIS_NO_FOLLOW_YAW,CHASSIS_OPEN
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && !CHASSIS_MODE_CONTROL->CHASSIS_TOP_KEY_T)
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) || CHASSIS_MODE_CONTROL->CHASSIS_TOP_KEY_T)
    {
        chassis_behaviour_mode = CHASSIS_TOP;
    }

    //when gimbal in some mode, such as init mode, chassis must's move
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }


    //add your own logic to enter the new mode


    //accord to beheviour mode, choose chassis control mode
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }

    /*添加的模式*/
    else if (chassis_behaviour_mode == CHASSIS_TOP)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TOP;
    }
}


/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector); 
    }

    /*添加*/
    else if(chassis_behaviour_mode == CHASSIS_TOP)
    {
        chassis_no_follow_yaw_control(vx_set,vy_set,angle_set,chassis_move_rc_to_vector);
    }

}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //swing angle is generated by sin function, swing_time is the input time of sin
    static fp32 swing_time = 0.0f;
    
    static fp32 swing_angle = 0.0f;
    //max_angle is the max angle that chassis will ratate
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //swing_time  plus the add_time in one control cycle
    static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    
    static uint8_t swing_flag = 0;

    //judge if swing
    if (0)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //judge if keyboard is controlling the chassis, if yes, reduce the max_angle
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    
    if (0)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //swing_time  range [0, 2*PI]
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }


    *angle_set = swing_angle;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
//    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

