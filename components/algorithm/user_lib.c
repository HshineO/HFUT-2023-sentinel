#include "user_lib.h"
#include "arm_math.h"

const fp32 k1=0.0761; //标准大气压25度下空气阻力系数

trajectory_type_t trajectory_cal;

/**
  * @brief          子弹飞行时间解算
  * @author         SJQ
  * @param[in]      x:瞄准时shooter_link下目标x坐标
  * @param[in]      vx:瞄准时shooter_link下目标x速度
  * @param[in]      v_x0:弹速水平分量
  * @retval         返回空
  */
fp32 get_fly_time(fp32 x,fp32 vx,fp32 v_x0)
{
   fp32 t=0;
   fp32 f_ti=0,df_ti=0; 
   for (int i = 0; i < 5; i++)
   {
       f_ti=log(k1*v_x0*t+1)/k1-x-vx*t;
       df_ti=v_x0/(k1*v_x0*t+1)-vx;
       t=t-f_ti/df_ti;
       /* code */
   }
   return t;
}
/**
 * @brief          解算期望云台角度（考虑子弹飞行时间）
 * @author         SJQ
 * @param[in]      trajectory_cal:弹道解算结构体
 * @retval         返回空
 */
void get_cmd_angle(trajectory_type_t *trajectory_cal)
{
	
   arm_power_f32(trajectory_cal->position_xy,2,&trajectory_cal->dis2);
   arm_sqrt_f32(trajectory_cal->dis2,&trajectory_cal->dis);
   trajectory_cal->theta_0 = atan2f(trajectory_cal->z,trajectory_cal->dis);
   trajectory_cal->alpha = atan2f(trajectory_cal->position_xy[1],trajectory_cal->position_xy[0]);
   //将目标的xyz速度转化为平行枪管与垂直枪管的速度
   trajectory_cal->vx = trajectory_cal->velocity[0] * arm_cos_f32(trajectory_cal->alpha)
                       + trajectory_cal->velocity[1] * arm_sin_f32(trajectory_cal->alpha);
   trajectory_cal->vy = - trajectory_cal->velocity[0] * arm_sin_f32(trajectory_cal->alpha)
                       + trajectory_cal->velocity[1] * arm_cos_f32(trajectory_cal->alpha);
   
   fp32 v_x0 = trajectory_cal->v0 * arm_cos_f32(trajectory_cal->theta_0);//水平方向弹速
   fp32 v_y0 = trajectory_cal->v0 * arm_sin_f32(trajectory_cal->theta_0);//竖直方向弹速
   
   trajectory_cal->fly_time = get_fly_time(trajectory_cal->dis,trajectory_cal->vx,v_x0);
   arm_power_f32(&trajectory_cal->fly_time,1,&trajectory_cal->fly_time2);
   trajectory_cal->h_r = trajectory_cal->z + trajectory_cal->velocity[2] * trajectory_cal->fly_time;
   //开始迭代
   trajectory_cal->theta_k = trajectory_cal->theta_0;
   trajectory_cal->k = 0;
   while (trajectory_cal->k < 10)
   {
        v_y0 = trajectory_cal->v0 * arm_sin_f32(trajectory_cal->theta_k);//竖直方向弹速
       trajectory_cal->h_k = v_y0 * trajectory_cal->fly_time - 0.5*9.8*trajectory_cal->fly_time2;
       trajectory_cal->err_k = trajectory_cal->h_r - trajectory_cal->h_k;
       trajectory_cal->theta_k += 0.1 * trajectory_cal->err_k;
       trajectory_cal->k++;
       if (trajectory_cal->err_k < 0.005) break;      
   }

   
   
   trajectory_cal->cmd_yaw = atan2f(trajectory_cal->position_xy[1] + trajectory_cal->velocity[1]*(trajectory_cal->fly_time + trajectory_cal->extra_delay_time),
                                   trajectory_cal->position_xy[0] + trajectory_cal->velocity[0]*(trajectory_cal->fly_time + trajectory_cal->extra_delay_time));
   trajectory_cal->cmd_pitch = trajectory_cal->theta_k;
}

//快速开方
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}
