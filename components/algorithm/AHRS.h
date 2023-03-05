#ifndef AHRS_H
#define AHRS_H

#include "AHRS_MiddleWare.h"


extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);

extern fp32 get_yaw(const fp32 quat[4]);

extern fp32 get_pitch(const fp32 quat[4]);

extern fp32 get_roll(const fp32 quat[4]);

extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern fp32 get_carrier_gravity(void);

#endif
