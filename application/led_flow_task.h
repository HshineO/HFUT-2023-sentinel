/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGB��Ч��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef LED_TRIGGER_TASK_H
#define LED_TRIGGER_TASK_H


#include "struct_typedef.h"

#include "main.h"
#include "can_Receive.h"
#include "remote_control.h"


void led_RGB_flow_task(void const * argument);
#if 0
#include "gimbal_Task.h"
#include "pid.h"

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
	uint16_t given_current;
	fp32 speed;
	fp32 speed_set;
	const motor_measure_t	*extra_3508_measure;
	pid_type_def motor_speed_pid;						       // ����ٶ�pid
}Extra_3508_Data_t;

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

void Motor_User_Init( User_Motor_t *User_Motor_InitType );
void Motor_User_Mode_Set( User_Motor_t *User_Motor_ModeTypedef );
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType );
void Motor_2006_Data_Processing( User_Motor_t *User_Motor_DataType );
void Extra_3508_Speed_PID( User_Motor_t *User_Motor_PIDType );
void Motor_2006_PID_Set( User_Motor_t *User_Motor_PIDType );

// ��չ����ƴ���
void ExBoard_Angle_Init_Mode( CAN_HandleTypeDef* CANx, uint16_t addr );
void ExBoard_Servo_Control_Mode( CAN_HandleTypeDef* CANx, uint16_t addr, uint16_t angle1, uint16_t angle2 );
void CAR_FORCE_RESET( void );

#endif

#endif
