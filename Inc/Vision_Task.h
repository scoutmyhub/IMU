#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "main.h"
#include "user_lib.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "gimbal_behaviour.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "Vision.h"

#include "struct_typedef.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"


#define VISION_HUART  huart1
typedef union 
{
	float f_data;
	uint8_t byte[4];
}FloatUChar_t;

/*发送数据*/
typedef struct
{
	uint8_t Length;
	uint8_t Fire_Speed;
	uint8_t Colour;
	uint8_t Mode;
	float Systime;
}Vision_Data_t;

typedef struct Info
{
	uint8_t FrameHeader;
	Vision_Data_t Vision_Data;
	uint8_t FrameTailer;
}Vision_Info;



/*接受数据*/
typedef struct
{
	uint8_t FrameHeader;
	uint8_t Length;
	uint8_t Distance;
	float Pitch_Offset;
	float Yaw_Offset;
	float Goal_Speed;
	float Timestamp;
	uint8_t Shoot_Instruct;
	uint8_t FrameTailer;
}Receive_Data_t;



typedef __packed struct
{
	char frame_head ;
	uint8_t a;
	char b;
	char c;

	float yaw_angle;
	float pitch_angle;
	uint8_t state;
	uint8_t mark;
	uint8_t anti_top;
	uint8_t color;
	uint8_t shoot;	
	int delta_x;
	int delta_y;
  uint8_t frame_tail;
}Vision_send_t;

typedef __packed struct
{
	  char frame_head;
	  float yaw_angle;
	  float pitch_angle;
	  float distance;
		float time;
	  char shot_flag;
	  char frame_tail;	
}VisionRecvData_t;


extern void vision_send_task(void const * argument);
extern void	LanggoUartFrameIRQHandler(UART_HandleTypeDef *huart);
extern VisionRecvData_t VisionRecvData;

extern void Vision_task(void const *pvParameters);

#endif
