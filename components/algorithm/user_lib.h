#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
//快速开方

typedef  struct
{
    fp32 velocity[3];//目标xyz速度
    fp32 vx;         //目标相对枪口方向的速度
	fp32 vy;
    fp32 alpha;      //目标初始航向角
    fp32 position_xy[2];//目标xy坐标
    fp32 z;         //目标z坐标
    fp32 fly_time;  //子弹飞行时间
    fp32 fly_time2;  //子弹飞行时间平方
    fp32 extra_delay_time ;
    fp32 cmd_yaw;
    fp32 cmd_pitch;
    fp32 v0;        //子弹射速
    fp32 theta_0;   //初始目标角度
    fp32 theta_k;   //迭代目标角度
    fp32 dis;       //目标距离
    fp32 dis2;       //目标距离平方
    fp32 err_k;      //迭代误差
    uint8_t k;      //迭代次数
    fp32 h_k;       //迭代高度
    fp32 h_r;       //目标真实高度
} trajectory_type_t;
extern trajectory_type_t trajectory_cal;
//弹道解算

extern fp32 invSqrt(fp32 num);

void get_cmd_angle(trajectory_type_t *trajectory_cal);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
extern fp32 sign(fp32 value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
