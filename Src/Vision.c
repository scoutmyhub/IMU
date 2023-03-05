//#include "Vision.h"
//#include "Vision_Task.h"
//#include "referee_lib.h"

//Vision_t *Vision;
//static uint8_t mode;
//uint8_t Get_Vision_Mode(void)
//{

//	if (Vision->Vision_rc_ctrl->mouse.press_r)
//	{
//		mode = ACT_NORMOL;
//		if (Vision->Vision_rc_ctrl->key.v & Vision_SMALL_BUFF_KEY)
//		{
//			mode = ACT_BIG_BUFF;
//		}
//		else if (Vision->Vision_rc_ctrl->key.v & Vision_BIG_BUFF_KEY)
//		{
//			mode = ACT_SMALL_BUFF;
//		}
//	}
//	else
//	{
//		mode = NO_VISION;
//	}

//	return mode;
//}
//uint8_t Get_Robot_Colour(void)
//{
//	static uint8_t res;
//	if (get_robot_id() > 10)
//	{
//		res = I_AM_BLUE;
//	}
//	else
//		res = I_AM_RED;

//	return res;
//}
//uint16_t Get_Fire1_Speed(void)
//{
//	return ID1_speed_limit();
//}

//uint16_t Get_Fire2_Speed(void)
//{
//	return ID2_speed_limit();
//}



