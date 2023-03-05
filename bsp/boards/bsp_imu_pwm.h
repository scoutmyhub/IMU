#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H
#include "struct_typedef.h"
#include "stdint.h"
#include "tim.h"
extern void imu_pwm_set(uint16_t pwm);

extern void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);
#endif
