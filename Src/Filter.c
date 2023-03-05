#include "Filter.h"

/// @brief һ�׵�ͨ�˲��ĳ�ʼ��
/// @param Low_Pass_Filter �ṹ��
/// @param frame_period ����ʱ��
/// @param Trust ��Ȩ����
void Low_Pass_Filter_Init(Low_Pass_Filter_t *Low_Pass_Filter, fp32 frame_period, const fp32 Trust)
{
    Low_Pass_Filter->frame_period = frame_period;
    Low_Pass_Filter->Trust = Trust;
    Low_Pass_Filter->input = 0.0f;
    Low_Pass_Filter->out = 0.0f;
}

/// @brief һ�׵�ͨ�˲������ͨ����Ȩ�õ������
/// @param Low_Pass_Filter �ṹ��
/// @param input ���
void Low_Pass_Filter_OUT(Low_Pass_Filter_t *Low_Pass_Filter, fp32 input)
{
    Low_Pass_Filter->input = input;
    Low_Pass_Filter->out = Low_Pass_Filter->frame_period / (Low_Pass_Filter->Trust + Low_Pass_Filter->frame_period) * Low_Pass_Filter->out 
                         + Low_Pass_Filter->Trust / (Low_Pass_Filter->Trust + Low_Pass_Filter->frame_period) * Low_Pass_Filter->input;
}





