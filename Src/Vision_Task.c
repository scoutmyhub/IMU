//#include "Vision_Task.h"
//#include "referee_lib.h"
//#include "Vision.h"
//#include "bsp_usart.h"
//#include "stdlib.h"
//#include "INS_task.h"
//#include "usart.h"
//#include "cmsis_os.h"
//#include "math.h"

//FloatUChar_t Qart[4];
//FloatUChar_t Gyro[3];
//FloatUChar_t Accel[3];
//void vision_send(void);

//extern fp32 gyro[3], accel[3], temp;
//extern fp32 INS_quat[4];

///**
// * @brief        Convert quaternion to eular angle
// */


//void Vision_task(void const *pvParameters)
//{
// 
//	while (1)
//	{
////		vision_send();
////		QuaternionToEularAngle(INS_quat, &myyaw, &mypitch, &myroll)	
//		vTaskDelay(5);
//	}
//}

//void vision_send(void)
//{
//	uint8_t length = 46;
//	uint8_t sv_buff[46];

//	Qart[0].f_data = (float)INS_quat[0];
//	Qart[1].f_data = (float)INS_quat[1];
//	Qart[2].f_data = (float)INS_quat[2];
//	Qart[3].f_data = (float)INS_quat[3];

//	Gyro[0].f_data = (float)INS_gyro[0];
//	Gyro[1].f_data = (float)INS_gyro[1];
//	Gyro[2].f_data = (float)INS_gyro[2];

//	Accel[0].f_data = (float)INS_accel[0];
//	Accel[1].f_data = (float)INS_accel[1];
//	Accel[2].f_data = (float)INS_accel[2];

//	// sv_buff[0] = Vision_send.frame_head;
//	// sv_buff[1] = Vision_send.a;
//	// sv_buff[2] = Vision_send.b;
//	// sv_buff[3] = Vision_send.c;
//	// sv_buff[4] = *TX_0.byte;
//	// sv_buff[5] = *(TX_0.byte + 1);
//	// sv_buff[6] = *(TX_0.byte + 2);
//	// sv_buff[7] = *(TX_0.byte + 3);
//	// sv_buff[8] = *TX_1.byte;
//	// sv_buff[9] = *(TX_1.byte + 1);
//	// sv_buff[10] = *(TX_1.byte + 2);
//	// sv_buff[11] = *(TX_1.byte + 3);
//	// sv_buff[12] = *TX_2.byte;
//	// sv_buff[13] = *(TX_2.byte + 1);
//	// sv_buff[14] = *(TX_2.byte + 2);
//	// sv_buff[15] = *(TX_2.byte + 3);
//	// sv_buff[16] = *TX_3.byte;
//	// sv_buff[17] = *(TX_3.byte + 1);
//	// sv_buff[18] = *(TX_3.byte + 2);
//	// sv_buff[19] = *(TX_3.byte + 3);
//	// sv_buff[20] = *(TX_gyro1.byte);
//	// sv_buff[21] = *(TX_gyro1.byte + 1);
//	// sv_buff[22] = *(TX_gyro1.byte + 2);
//	// sv_buff[23] = *(TX_gyro1.byte + 3);
//	// sv_buff[24] = *(TX_gyro2.byte);
//	// sv_buff[25] = *(TX_gyro2.byte + 1);
//	// sv_buff[26] = *(TX_gyro2.byte + 2);
//	// sv_buff[27] = *(TX_gyro2.byte + 3);
//	// sv_buff[28] = *(TX_gyro3.byte);
//	// sv_buff[29] = *(TX_gyro3.byte + 1);
//	// sv_buff[30] = *(TX_gyro3.byte + 2);
//	// sv_buff[31] = *(TX_gyro3.byte + 3);
//	// sv_buff[32] = *(TX_acc1.byte);
//	// sv_buff[33] = *(TX_acc1.byte + 1);
//	// sv_buff[34] = *(TX_acc1.byte + 2);
//	// sv_buff[35] = *(TX_acc1.byte + 3);
//	// sv_buff[36] = *(TX_acc2.byte);
//	// sv_buff[37] = *(TX_acc2.byte + 1);
//	// sv_buff[38] = *(TX_acc2.byte + 2);
//	// sv_buff[39] = *(TX_acc2.byte + 3);
//	// sv_buff[40] = *(TX_acc3.byte);
//	// sv_buff[41] = *(TX_acc3.byte + 1);
//	// sv_buff[42] = *(TX_acc3.byte + 2);
//	// sv_buff[43] = *(TX_acc3.byte + 3);
//	// sv_buff[44] = Vision_send.shoot;
//	// sv_buff[45] = Vision_send.frame_tail;

//	HAL_UART_Transmit(&VISION_HUART, (uint8_t *)sv_buff, length, 100);
//}
