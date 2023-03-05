#include "bsp_imu_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}


void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}

