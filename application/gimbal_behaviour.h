#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"

#include "gimbal_task.h"
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0


typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,     
} gimbal_behaviour_e;


extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);

extern bool_t gimbal_cmd_to_chassis_stop(void);

extern bool_t gimbal_cmd_to_shoot_stop(void);

#endif
