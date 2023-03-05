#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

/*打开弹仓盖*//*NOW 开 ；NEXT 关*/
#define SHOOT_OPEN_CLIP KEY_PRESSED_OFFSET_B

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500
uint8_t LAST_SHOOT_OPEN_CLIP = 0;
const RC_ctrl_t *servo_rc;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();

    while(1)
    {
        if ( (servo_rc->key.v & SHOOT_OPEN_CLIP && !(LAST_SHOOT_OPEN_CLIP & SHOOT_OPEN_CLIP)) /*||遥控器的条件*/ )
        {
            /*引脚生成PWM*/
        }
        else
        {
            /*复位*/
        }
        LAST_SHOOT_OPEN_CLIP = servo_rc->key.v;
        
        osDelay(10);
    }
}


//   if ((Shoot_motor.RC_ctrl->key.v & SHOOT_OPEN_CLIP) && (!(Shoot_motor.Last_Control_SET.LAST_SHOOT_SWITCH_CLIP_KEY & SHOOT_OPEN_CLIP)))
//   {
//     Shoot_motor.Control_FALG.SHOOT_OPEN_CLIP_FLAG = !Shoot_motor.Control_FALG.SHOOT_OPEN_CLIP_FLAG;
//   }
//   Shoot_motor.Last_Control_SET.LAST_SHOOT_SWITCH_CLIP_KEY = Shoot_motor.RC_ctrl->key.v;

