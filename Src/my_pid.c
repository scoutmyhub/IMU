#include "my_pid.h"
#define ABS(x)		((x>0)? x: -x)   //源于老代码
//绝对限制
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }




pid_t pid_poke = {0};
pid_t pid_poke_omg = {0};


float pid_calc_old(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
			return 0;
		if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
			return 0;
    
    if(pid->pid_mode == POSITION_PID) //???p
    {   
//			  if(rc.ch3>-3&&rc.ch3<3)	
//				pid->err[NOW]=0;
//        pid->iout=0;	
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//???P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}//源于老代码

static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}//源于老代码




static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}



void PID_struct_init
	(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}//源于老代码




/**********************视觉PID**********************/

float Vision_AutoTracPID(positionpid_t *pid_t, float target, float measured)
{
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉YAW轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉YAW轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉PITCH轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉PITCH轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/**************************************************/
























