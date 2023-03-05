#include "referee.h"

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}

uint8_t get_robot_id(void)
{
	return robot_state.robot_id;
}

uint8_t get_robot_level(void)
{
	return robot_state.robot_level;
}
uint16_t get_robot_remain_HP(void)
{
	return robot_state.remain_HP;
}


uint16_t get_robot_max_HP(void)
{
	return robot_state.max_HP;
}


uint16_t ID1_cooling_rate(void)
{
	return robot_state.shooter_id1_17mm_cooling_rate;
}
uint16_t ID1_cooling_limit(void)
{
	return robot_state.shooter_id1_17mm_cooling_limit;
}
uint16_t ID1_speed_limit(void)
{
	return robot_state.shooter_id1_17mm_speed_limit;
}


uint16_t ID2_cooling_rate(void)
{
	return robot_state.shooter_id2_17mm_cooling_rate;
}
uint16_t ID2_cooling_limit(void)
{
	return robot_state.shooter_id2_17mm_cooling_limit;
}
uint16_t ID2_speed_limit(void)
{
	return robot_state.shooter_id2_17mm_speed_limit;
}



uint8_t robot_buff(void)
{
	return buff_musk_t.power_rune_buff;
}

uint16_t Chassis_Power_Limit(void)
{
  return robot_state.chassis_power_limit;
}


