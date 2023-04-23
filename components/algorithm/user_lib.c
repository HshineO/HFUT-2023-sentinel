#include "user_lib.h"
#include "arm_math.h"

const fp32 k1=0.0761; //��׼����ѹ25���¿�������ϵ��

trajectory_type_t trajectory_cal;

/**
  * @brief          �ӵ�����ʱ�����
  * @author         SJQ
  * @param[in]      x:��׼ʱshooter_link��Ŀ��x����
  * @param[in]      vx:��׼ʱshooter_link��Ŀ��x�ٶ�
  * @param[in]      v_x0:����ˮƽ����
  * @retval         ���ؿ�
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
 * @brief          ����������̨�Ƕȣ������ӵ�����ʱ�䣩
 * @author         SJQ
 * @param[in]      trajectory_cal:��������ṹ��
 * @retval         ���ؿ�
 */
void get_cmd_angle(trajectory_type_t *trajectory_cal)
{
	
   arm_power_f32(trajectory_cal->position_xy,2,&trajectory_cal->dis2);
   arm_sqrt_f32(trajectory_cal->dis2,&trajectory_cal->dis);
   trajectory_cal->theta_0 = atan2f(trajectory_cal->z,trajectory_cal->dis);
   trajectory_cal->alpha = atan2f(trajectory_cal->position_xy[1],trajectory_cal->position_xy[0]);
   //��Ŀ���xyz�ٶ�ת��Ϊƽ��ǹ���봹ֱǹ�ܵ��ٶ�
   trajectory_cal->vx = trajectory_cal->velocity[0] * arm_cos_f32(trajectory_cal->alpha)
                       + trajectory_cal->velocity[1] * arm_sin_f32(trajectory_cal->alpha);
   trajectory_cal->vy = - trajectory_cal->velocity[0] * arm_sin_f32(trajectory_cal->alpha)
                       + trajectory_cal->velocity[1] * arm_cos_f32(trajectory_cal->alpha);
   
   fp32 v_x0 = trajectory_cal->v0 * arm_cos_f32(trajectory_cal->theta_0);//ˮƽ������
   fp32 v_y0 = trajectory_cal->v0 * arm_sin_f32(trajectory_cal->theta_0);//��ֱ������
   
   trajectory_cal->fly_time = get_fly_time(trajectory_cal->dis,trajectory_cal->vx,v_x0);
   arm_power_f32(&trajectory_cal->fly_time,1,&trajectory_cal->fly_time2);
   trajectory_cal->h_r = trajectory_cal->z + trajectory_cal->velocity[2] * trajectory_cal->fly_time;
   //��ʼ����
   trajectory_cal->theta_k = trajectory_cal->theta_0;
   trajectory_cal->k = 0;
   while (trajectory_cal->k < 10)
   {
        v_y0 = trajectory_cal->v0 * arm_sin_f32(trajectory_cal->theta_k);//��ֱ������
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

//���ٿ���
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
  * @brief          б��������ʼ��
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      ���ֵ
  * @param[in]      ��Сֵ
  * @retval         ���ؿ�
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
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
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
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//��������
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

//�жϷ���λ
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

//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//ѭ���޷�����
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

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}
