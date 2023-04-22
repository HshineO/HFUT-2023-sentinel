/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


/****************** 3508 PID值设置 ********************/
#define EXTRA_3508_SPEED_KP								    30.0f
#define EXTRA_3508_SPEED_KI								    2.0f
#define EXTRA_3508_SPEED_KD								    0.0f

#define EXTRA_3508_SPEED_PID_MAXOUT						16000.0f			// 3508最大能发送的电流
#define EXTRA_3508_SPEED_PID_MAXIOUT					20000.0f

/****************** 2006 PID值设置 ********************/
#define MOTOR_2006_SPEED_KP								20.0f
#define MOTOR_2006_SPEED_KI								1.0f
#define MOTOR_2006_SPEED_KD								0.0f

#define MOTOR_2006_SPEED_PID_MAXOUT						6000.0f				// 2006最大能发送的电流
#define MOTOR_2006_SPEED_PID_MAXIOUT					10000.0f

// 电机编码器最大值和最小值( 3508, 2006 )
#define MOTOR_USER_MAX_ENCODER							  8191
#define MOTOR_USER_MIN_ENCODER							  0
#define MOTOR_USER_HALF_ENCODER							  4096

//#define MOTOR_2006_ENCODER_A_ROUND					286720
#define MOTOR_2006_ENCODER_A_ROUND						294912
#define MOTOR_2006_ENCODER_HALF_ROUND					(MOTOR_2006_ENCODER_A_ROUND/2)
#define MOTOR_2006_ENCODER_ONE_BULLET					(MOTOR_2006_ENCODER_A_ROUND/8)
#define MOTOR_2006_ENCODER_ONE_SHOOT					(MOTOR_2006_ENCODER_A_ROUND/8)
//#define MOTOR_2006_ENCODER_ONE_BULLET					49152
#define MOTOR_2006_ENCODER_ROUND_ERROR				 0
#define MOTOR_2006_ENCODER_COUNT_MAX					 36

/****************** 3508 电机的旋转速度 ********************/
#define EXTRA_3508_ROTATE_SPEED_30						  	6800.0f//6800.0f
#define EXTRA_3508_ROTATE_SPEED_18						  	4800.0f
#define EXTRA_3508_ROTATE_SPEED_15						  	4100.0f

/****************** 2006 电机的旋转速度 ********************/
#define MOTOR_2006_SLOW_ROTATE_SPEED					1000.0f
#define MOTOR_2006_QUICK_ROTATE_SPEED					3000.0f		// 只用到这个，慢转没有用到

/****************** 电机的旋转方向 ********************/
#define EXTRA_3508_high_L_ROTATE_DIR							1
#define EXTRA_3508_low_L_ROTATE_DIR							1
#define EXTRA_3508_high_R_ROTATE_DIR							0
#define EXTRA_3508_low_R_ROTATE_DIR							0
#define MOTOR_2006_L_ROTATE_DIR							  0
#define MOTOR_2006_R_ROTATE_DIR							  0



/****************** 模式按键 ********************/
#define CONTROL_GUARD_TO_ATTACK			KEY_PRESSED_OFFSET_Q
#define CONTROL_GUARD_TO_FLEE			KEY_PRESSED_OFFSET_E

/****************** 电机堵转参数 ********************/
#define MOTOR_2006_LOCKED_ROTOR_ERROR			2
#define MOTOR_2006_LOCKED_ROTOR_SHORT			50
#define MOTOR_2006_LOCKED_ROTOR_LONG			300
#define MOTOR_2006_LOCKED_ROTOR_REVERSAL		200
#define MOTOR_2006_LOCKED_ROTOR_MAX				500
#define MOTOR_2006_LOCKED_ROTOR_MIN				0

#define	NORMAL_MODE 					      0x00			// 正常模式，正常工作
#define	MPU6050_ANGLE_INIT_MODE			0x01			// 陀螺仪初始化
#define	SERVO_ANGLE_SET_MODE			  0x02			// 舵机角度设置

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE     80

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
	uint16_t given_current;
	fp32 speed;
	fp32 speed_set;
	const motor_measure_t	*extra_3508_measure;
	pid_type_def motor_speed_pid;						       // 电机速度pid
}Extra_3508_Data_t;

typedef enum
{
	MOTOR_USER_STOP_MODE,	  // 停止
	MOTOR_GET_READY_MODE,	  // 准备射击
	MOTOR_SHOOT_MODE,		    // 射击子弹
	MOTOR_DONE_MODE,		    // 射击完成
	CLEAR_BULLET_PRE_MODE,	// 准备清空所有子弹模块
	CLEAR_BULLET_STA_MODE,	// 开始清空所有子弹模块
	MOTOR_LOCKED_ROTOR,		// 堵转
}User_Mode_e;

typedef struct
{
	int16_t encoder_pos_count;
	int16_t encoder_pos_ecd;
	int16_t encoder_pos_ecd_last;
	int16_t given_current;
	int32_t encoder_pos_all;
	int32_t encoder_pos_all_set;
	fp32 speed;
	fp32 speed_set;
	const motor_measure_t	*motor_2006_measure;
	pid_type_def motor_speed_pid;						        // 电机速度pid
}One_2006_Motor_Data_t;

typedef struct
{
	const RC_ctrl_t *user_motor_RC;				 // 底盘使用的遥控器指针
	Extra_3508_Data_t motor_extra_3508_high_l;		  // 摩擦轮电机数据
	Extra_3508_Data_t motor_extra_3508_low_l;		  // 摩擦轮电机数据
	Extra_3508_Data_t motor_extra_3508_high_r;     	  // 摩擦轮电机数据
	Extra_3508_Data_t motor_extra_3508_low_r;		  // 摩擦轮电机数据
	One_2006_Motor_Data_t motor_l_2006;		// 拨码盘电机	
	One_2006_Motor_Data_t motor_r_2006;		// 拨码盘电机	
	
	User_Mode_e motor_mode_l;
	int16_t locked_rotor_time_l;					// 拨弹轮堵转时间检测
	int16_t reversal_time_l;						// 反转时间计算

	User_Mode_e motor_mode_r;
	int16_t locked_rotor_time_r;					// 拨弹轮堵转时间检测
	int16_t reversal_time_r;						// 反转时间计算
}User_Motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
void Motor_User_Init( User_Motor_t *User_Motor_InitType );
void Motor_User_Mode_Set( User_Motor_t *User_Motor_ModeTypedef );
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType );
void Motor_2006_Data_Processing( User_Motor_t *User_Motor_DataType );
void Extra_3508_Speed_PID( User_Motor_t *User_Motor_PIDType );
void Motor_2006_PID_Set( User_Motor_t *User_Motor_PIDType );

#endif /* __SHOOT_TASK_H */
