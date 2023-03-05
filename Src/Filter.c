#include "Filter.h"

/// @brief 一阶低通滤波的初始化
/// @param Low_Pass_Filter 结构体
/// @param frame_period 采样时间
/// @param Trust 加权比例
void Low_Pass_Filter_Init(Low_Pass_Filter_t *Low_Pass_Filter, fp32 frame_period, const fp32 Trust)
{
    Low_Pass_Filter->frame_period = frame_period;
    Low_Pass_Filter->Trust = Trust;
    Low_Pass_Filter->input = 0.0f;
    Low_Pass_Filter->out = 0.0f;
}

/// @brief 一阶低通滤波输出，通过加权得到新输出
/// @param Low_Pass_Filter 结构体
/// @param input 输出
void Low_Pass_Filter_OUT(Low_Pass_Filter_t *Low_Pass_Filter, fp32 input)
{
    Low_Pass_Filter->input = input;
    Low_Pass_Filter->out = Low_Pass_Filter->frame_period / (Low_Pass_Filter->Trust + Low_Pass_Filter->frame_period) * Low_Pass_Filter->out 
                         + Low_Pass_Filter->Trust / (Low_Pass_Filter->Trust + Low_Pass_Filter->frame_period) * Low_Pass_Filter->input;
}





