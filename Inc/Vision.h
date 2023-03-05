#ifndef VISION_H
#define VISION_H


#include "main.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "remote_control.h"

#define  Vision_SMALL_BUFF_KEY  KEY_PRESSED_OFFSET_C
#define Vision_BIG_BUFF_KEY KEY_PRESSED_OFFSET_V

#define MAX_DMA_COUNT 100

/**
 *	@brief	自身颜色
 */
typedef enum
{
    UNKNOWN = 0,
    I_AM_BLUE = 1,
    I_AM_RED = 2,
} Color_t;

/*INS_OFFSET 偏移量*/
typedef enum
{
    INS_Pitch = 0,
    INS_Yaw = 1,
    INS_Roll = 3,
} INS_Offset_t;

/*控制行为*/
typedef enum
{
    ACT_Err = 0,
    ACT_NORMOL = 1, //普通模式
    ACT_BIG_BUFF,   //大符模式
    ACT_SMALL_BUFF, //小符模式
    ACT_AUTO_AIM,   //自瞄模式
    ACT_MODE_CNT,
} Action_t;

/*视觉行为命令码*/
typedef enum
{
    NO_VISION = 0x00,          //松开手时
    CMD_AIM_AUTO = 0x01,       // 自瞄
    CMD_AIM_SMALL_BUFF = 0x02, // 识别小符
    CMD_AIM_BIG_BUFF = 0x03,   // 识别大符
    CMD_AIM_SENTRY = 0x04,     // 击打哨兵
    CMD_AIM_BASE = 0x05,       // 吊射基地
} Vision_Cmd_Id_t;




typedef struct
{
        const RC_ctrl_t *Vision_rc_ctrl;

}Vision_t;


extern uint8_t Get_Vision_Mode(void);
extern uint16_t Get_Fire2_Speed(void);
extern uint16_t Get_Fire2_Speed(void);
extern uint8_t Get_Robot_Colour(void);


#endif

