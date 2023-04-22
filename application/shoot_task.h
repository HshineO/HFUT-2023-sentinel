/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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


/****************** 3508 PIDֵ���� ********************/
#define EXTRA_3508_SPEED_KP								    30.0f
#define EXTRA_3508_SPEED_KI								    2.0f
#define EXTRA_3508_SPEED_KD								    0.0f

#define EXTRA_3508_SPEED_PID_MAXOUT						16000.0f			// 3508����ܷ��͵ĵ���
#define EXTRA_3508_SPEED_PID_MAXIOUT					20000.0f

/****************** 2006 PIDֵ���� ********************/
#define MOTOR_2006_SPEED_KP								20.0f
#define MOTOR_2006_SPEED_KI								1.0f
#define MOTOR_2006_SPEED_KD								0.0f

#define MOTOR_2006_SPEED_PID_MAXOUT						6000.0f				// 2006����ܷ��͵ĵ���
#define MOTOR_2006_SPEED_PID_MAXIOUT					10000.0f

// ������������ֵ����Сֵ( 3508, 2006 )
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

/****************** 3508 �������ת�ٶ� ********************/
#define EXTRA_3508_ROTATE_SPEED_30						  	6800.0f//6800.0f
#define EXTRA_3508_ROTATE_SPEED_18						  	4800.0f
#define EXTRA_3508_ROTATE_SPEED_15						  	4100.0f

/****************** 2006 �������ת�ٶ� ********************/
#define MOTOR_2006_SLOW_ROTATE_SPEED					1000.0f
#define MOTOR_2006_QUICK_ROTATE_SPEED					3000.0f		// ֻ�õ��������תû���õ�

/****************** �������ת���� ********************/
#define EXTRA_3508_high_L_ROTATE_DIR							1
#define EXTRA_3508_low_L_ROTATE_DIR							1
#define EXTRA_3508_high_R_ROTATE_DIR							0
#define EXTRA_3508_low_R_ROTATE_DIR							0
#define MOTOR_2006_L_ROTATE_DIR							  0
#define MOTOR_2006_R_ROTATE_DIR							  0



/****************** ģʽ���� ********************/
#define CONTROL_GUARD_TO_ATTACK			KEY_PRESSED_OFFSET_Q
#define CONTROL_GUARD_TO_FLEE			KEY_PRESSED_OFFSET_E

/****************** �����ת���� ********************/
#define MOTOR_2006_LOCKED_ROTOR_ERROR			2
#define MOTOR_2006_LOCKED_ROTOR_SHORT			50
#define MOTOR_2006_LOCKED_ROTOR_LONG			300
#define MOTOR_2006_LOCKED_ROTOR_REVERSAL		200
#define MOTOR_2006_LOCKED_ROTOR_MAX				500
#define MOTOR_2006_LOCKED_ROTOR_MIN				0

#define	NORMAL_MODE 					      0x00			// ����ģʽ����������
#define	MPU6050_ANGLE_INIT_MODE			0x01			// �����ǳ�ʼ��
#define	SERVO_ANGLE_SET_MODE			  0x02			// ����Ƕ�����

//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж�
#define PRESS_LONG_TIME             400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//�����ٶ�
#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//�����ֵ��PID
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
	pid_type_def motor_speed_pid;						       // ����ٶ�pid
}Extra_3508_Data_t;

typedef enum
{
	MOTOR_USER_STOP_MODE,	  // ֹͣ
	MOTOR_GET_READY_MODE,	  // ׼�����
	MOTOR_SHOOT_MODE,		    // ����ӵ�
	MOTOR_DONE_MODE,		    // ������
	CLEAR_BULLET_PRE_MODE,	// ׼����������ӵ�ģ��
	CLEAR_BULLET_STA_MODE,	// ��ʼ��������ӵ�ģ��
	MOTOR_LOCKED_ROTOR,		// ��ת
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
	pid_type_def motor_speed_pid;						        // ����ٶ�pid
}One_2006_Motor_Data_t;

typedef struct
{
	const RC_ctrl_t *user_motor_RC;				 // ����ʹ�õ�ң����ָ��
	Extra_3508_Data_t motor_extra_3508_high_l;		  // Ħ���ֵ������
	Extra_3508_Data_t motor_extra_3508_low_l;		  // Ħ���ֵ������
	Extra_3508_Data_t motor_extra_3508_high_r;     	  // Ħ���ֵ������
	Extra_3508_Data_t motor_extra_3508_low_r;		  // Ħ���ֵ������
	One_2006_Motor_Data_t motor_l_2006;		// �����̵��	
	One_2006_Motor_Data_t motor_r_2006;		// �����̵��	
	
	User_Mode_e motor_mode_l;
	int16_t locked_rotor_time_l;					// �����ֶ�תʱ����
	int16_t reversal_time_l;						// ��תʱ�����

	User_Mode_e motor_mode_r;
	int16_t locked_rotor_time_r;					// �����ֶ�תʱ����
	int16_t reversal_time_r;						// ��תʱ�����
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

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
void Motor_User_Init( User_Motor_t *User_Motor_InitType );
void Motor_User_Mode_Set( User_Motor_t *User_Motor_ModeTypedef );
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType );
void Motor_2006_Data_Processing( User_Motor_t *User_Motor_DataType );
void Extra_3508_Speed_PID( User_Motor_t *User_Motor_PIDType );
void Motor_2006_PID_Set( User_Motor_t *User_Motor_PIDType );

#endif /* __SHOOT_TASK_H */
