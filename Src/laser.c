#include "laser.h"
void laser_on(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
}

void laser_off(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
}




