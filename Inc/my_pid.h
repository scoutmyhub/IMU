#ifndef MY_PID__H
#define MY_PID__H
#include "main.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "arm_math.h"
#include "user_lib.h"





typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//???,??NOW, LAST, LLAST???
    float get[3];				//???
    float err[3];				//??
	
    
    float pout;							//p??
    float iout;							//i??
    float dout;							//d??
    
    float pos_out;						//???????
    float last_pos_out;				//????
    float delta_u;						//?????
    float delta_out;					//??????? = last_delta_out + delta_u
    float last_delta_out;
    
		float max_err;
		float deadband;						//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//????
    uint32_t IntegralLimit;		//????
    
    void (*f_param_init)(struct __pid_t *pid,  //PID?????
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid??????
//    float (*f_cal_pid)(struct __pid_t *pid, float get, float set);   			//pid??
//		float (*f_cal_sp_pid)(struct __pid_t* pid, float get, float set, float gyro);
}pid_t;
//PID老代码的应用
enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};//老代码


typedef struct positionpid_t
{
    float Target;     //设定目标值
    float Measured;   //测量值
    float err;        //本次偏差值
    float err_last;   //上一次偏差
    float err_change; //误差变化率
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;               //各部分输出值
    float pwm;                 //pwm输出
    float MaxOutput;           //输出限幅
    float Integral_Separation; //积分分离阈值
    float IntegralLimit;       //积分限幅
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;


extern float pid_calc_old(pid_t* pid, float get, float set);
extern void PID_struct_init
	(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);

extern pid_t pid_poke;
extern pid_t pid_poke_omg;


/*视觉PID*/
extern float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured);



#endif




