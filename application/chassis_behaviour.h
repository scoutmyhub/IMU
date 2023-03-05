#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"





typedef enum
{
    CHASSIS_ZERO_FORCE,                  // chassis will be like no power
    CHASSIS_NO_MOVE,                     // chassis will be stop
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  // chassis will follow gimbal, usually in infantry
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, // chassis will follow chassis yaw angle, usually in engineer,
                                         // because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
    CHASSIS_NO_FOLLOW_YAW, // chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
    CHASSIS_TOP,                   /*小陀螺*/
    CHASSIS_OPEN // the value of remote control will mulitiply a value, get current value that will be sent to can bus
} chassis_behaviour_e;

/*在CHASSIS_OPEN mode下，遥控器的比例*/
#define CHASSIS_OPEN_RC_SCALE 10

typedef struct
{
    uint8_t CHASSIS_TOP_KEY_T;
    uint8_t LAST_CHASSIS_TOP_KEY;
}CHASSIS_MODE_CONTROL_T;



/**
 * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
 * @param[in]      chassis_move_mode: chassis data
 * @retval         none
 */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
 * @brief          set control set-point. three movement param, according to difference control mode,
 *                 will control corresponding movement.in the function, usually call different control function.
 * @param[out]     vx_set, usually controls vertical speed.
 * @param[out]     vy_set, usually controls horizotal speed.
 * @param[out]     wz_set, usually controls rotation speed.
 * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
 * @retval         none
 */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
