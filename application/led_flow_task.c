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
#include "led_flow_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"

#include "INS_Task.h"
#include "gimbal_task.h"
#include "usb_task.h"
#include "referee.h"
#include "math.h"
#include "cmsis_armcc.h"

extern ext_game_robot_state_t robot_state;
extern ext_shoot_data_t shoot_data_t;

//#define RGB_FLOW_COLOR_CHANGE_TIME  1000
//#define RGB_FLOW_COLOR_LENGHT   6
////blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
////�� -> ��(��) -> �� -> ��(��) -> �� -> ��(��) -> ��
//uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};


/**
  * @brief          led RGB����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
//void led_RGB_flow_task(void const *argument)
//{
//  uint16_t i, j;
//  fp32 delta_alpha, delta_red, delta_green, delta_blue;
//  fp32 alpha, red, green, blue;
//  uint32_t aRGB;

//  while(1)
//  {
//    for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
//    {
//      alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
//      red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
//      green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
//      blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

//      delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
//      delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
//      delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
//      delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

//      delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
//      delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
//      delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
//      delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
//      for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
//      {
//        alpha += delta_alpha;
//        red += delta_red;
//        green += delta_green;
//        blue += delta_blue;

//        aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
//        aRGB_led_show(aRGB);
//        osDelay(1);
//      }
//    }
//  }
//}

User_Motor_t user_motor;

static fp32 Extra_3508_SetCurrent_L = 0, Extra_3508_SetCurrent_R = 0, Motor_2006_Motor = 0;
volatile extern int16_t Yaw_Can_Set_Current, Pitch_Can_Set_Current;
extern uint16_t reset_count;
extern ext_robot_hurt_t robot_hurt_t;

fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

// ����ϵͳ����
extern ext_game_robot_state_t robot_state;

void led_RGB_flow_task(void const *argument)
{
	vTaskDelay(50);
	Motor_User_Init(&user_motor);

  while(1)
  {
	  //if ( robot_state.remain_HP )	// ������
	  {
		Motor_User_Mode_Set(&user_motor);
		Extra_3508_Data_Processing(&user_motor);
		Extra_3508_Speed_PID(&user_motor);
		Motor_2006_Data_Processing(&user_motor);
		Motor_2006_PID_Set(&user_motor);

		Extra_3508_SetCurrent_L = user_motor.motor_extra_3508_l.given_current;
		Extra_3508_SetCurrent_R = user_motor.motor_extra_3508_r.given_current;
		Motor_2006_Motor = user_motor.motor_2006.given_current;

		CAN_CMD_Extra3508( 0 , Motor_2006_Motor,Extra_3508_SetCurrent_L, Extra_3508_SetCurrent_R );
	  }
	  //else	// ������˼���Ѿ�׳������
	  //{
		  
	  //}
		
//	  usb_printf("%6d,%6d.", user_motor.motor_extra_3508_l.extra_3508_measure->speed_rpm, user_motor.motor_extra_3508_r.extra_3508_measure->speed_rpm );
		vTaskDelay(2);	
	}
}
// �����3508������ƴ���
void Motor_User_Init( User_Motor_t *User_Motor_InitType )
{
	const static fp32 extra_3508_speed_pid[3] = { EXTRA_3508_SPEED_KP, EXTRA_3508_SPEED_KI, EXTRA_3508_SPEED_KD };
	const static fp32 motor_2006_speed_pid[3] = { MOTOR_2006_SPEED_KP, MOTOR_2006_SPEED_KI, MOTOR_2006_SPEED_KD };
	
	// ģʽ��ʼ��
	User_Motor_InitType->motor_mode = MOTOR_USER_STOP_MODE;
	
	// ң��ָ���ʼ��
	User_Motor_InitType->user_motor_RC = get_remote_control_point();
	
	// 2006���������ʼ��
	User_Motor_InitType->motor_2006.encoder_pos_count = 1;
	User_Motor_InitType->motor_2006.encoder_pos_all = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_2006.encoder_pos_all_set = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_2006.encoder_pos_ecd = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_2006.encoder_pos_ecd_last = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	
	// 3508���������ʼ��
	User_Motor_InitType->motor_extra_3508_l.given_current = 0;
	User_Motor_InitType->motor_extra_3508_l.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_l.speed = 0;
	User_Motor_InitType->motor_extra_3508_r.given_current = 0;
	User_Motor_InitType->motor_extra_3508_r.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_r.speed = 0;
	
	// 3508 PID ��ʼ��
	User_Motor_InitType->motor_extra_3508_l.extra_3508_measure = get_shoot_motor_point(1);
	PID_init( &User_Motor_InitType->motor_extra_3508_l.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	User_Motor_InitType->motor_extra_3508_r.extra_3508_measure = get_shoot_motor_point(2);
	PID_init( &User_Motor_InitType->motor_extra_3508_r.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	// 2006 PID ��ʼ��
	User_Motor_InitType->motor_2006.motor_2006_measure = get_shoot_motor_point(0);
	PID_init( &User_Motor_InitType->motor_2006.motor_speed_pid, PID_POSITION, motor_2006_speed_pid, MOTOR_2006_SPEED_PID_MAXOUT, MOTOR_2006_SPEED_PID_MAXIOUT );
	
	User_Motor_InitType->locked_rotor_time = 0;
	User_Motor_InitType->reversal_time = 0;
}
void Motor_User_Mode_Set( User_Motor_t *User_Motor_ModeTypedef )
{
	static uint16_t shoot_count = 0;
	
	if(switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]))// )	
		{
			if( (User_Motor_ModeTypedef->motor_mode==MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode==MOTOR_DONE_MODE) )//����ע������
			{
				User_Motor_ModeTypedef->motor_mode = MOTOR_GET_READY_MODE;
			}
		}
		else if( switch_is_mid(User_Motor_ModeTypedef->user_motor_RC->rc.s[1] ))//
		{
			User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
		}
		else
		{
			shoot_count = 0;
		}
	// ң�����õ��ģʽ
//	if( switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]) )	
//	{
//		shoot_count++;
//		
//		if( shoot_count == 10 )		User_Motor_ModeTypedef->motor_mode = MOTOR_GET_READY_MODE;	// ���䵯��һ��
////		if( shoot_count == 8000 )	shoot_count = 8000, User_Motor_ModeTypedef->motor_mode = CLEAR_BULLET_PRE_MODE;	// ׼����������ӵ�
//		if( shoot_count > 8020 )	shoot_count = 8020;
//	}
//	else
//	{
//		shoot_count = 0;
//	}
//	if( switch_is_mid(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]) )
//	{
//		User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
//	}
	// else if( switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]) )	// ʹ�ü��̿���ģʽ
	// {
	// 	// �������õ��ģʽ
	// 	if( (User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_SHIFT_CTRL) == KEY_PRESSED_OFFSET_SHIFT_CTRL )	// shift��ctrl������
	// 	{
	// 		if( User_Motor_ModeTypedef->user_motor_RC->key.v & CONTROL_GUARD_TO_FLEE )	// �����ڱ�����
	// 		{
	// 		}
	// 		else if( User_Motor_ModeTypedef->user_motor_RC->key.v & CONTROL_GUARD_TO_ATTACK )		// �����ڱ�����
	// 		{
	// 		}
	// 	}
	// 	else if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )	// ֻ��shift����
	// 	{
			
	// 	}
	// 	else if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_CTRL )	// ֻ��ctrl������
	// 	{
			
	// 	}
	// 	// �����
	// 	if( User_Motor_ModeTypedef->user_motor_RC->mouse.press_l )
	// 	{
	// 		if( (User_Motor_ModeTypedef->motor_mode==MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode==MOTOR_DONE_MODE) )
	// 		{
	// 			User_Motor_ModeTypedef->motor_mode = MOTOR_GET_READY_MODE;
	// 		}
	// 	}
	// 	if( User_Motor_ModeTypedef->user_motor_RC->mouse.press_r )
	// 	{
	// 	}
		
	// 	if( reset_count >= 5000 )
	// 	{
	// 	  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_CTRL )	// ctrl���£�ǿ�и�λ��ť
	// 	  {
	// 		  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )	// shift���£�ǿ�и�λ��ť
	// 		  {
	// 			  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_A )	// A���£�ǿ�и�λ��ť
	// 			  {
	// 				  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_D )	// D���£�ǿ�и�λ��ť
	// 				  {
	// 					  CAR_FORCE_RESET();
	// 				  }
	// 			  }
	// 		  }
	// 	  }
	// 	}
	// }
	// if ( switch_is_down(User_Motor_ModeTypedef->user_motor_RC->rc.s[0]) )	// ���е��ͣת����������
	// {
	// 	User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
	// }
}
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType )
{
	static uint16_t shoot15_speed = 0, shoot18_speed = 0, shoot30_speed = 0;
	//fp32 speed_limit = robot_state.shooter_id1_17mm_speed_limit;
	
	// ����Ħ����3508������ݴ���
	User_Motor_DataType->motor_extra_3508_l.speed = User_Motor_DataType->motor_extra_3508_l.extra_3508_measure->speed_rpm;
	User_Motor_DataType->motor_extra_3508_r.speed = User_Motor_DataType->motor_extra_3508_r.extra_3508_measure->speed_rpm;
	
	
	
//	if( User_Motor_DataType->motor_mode == MOTOR_USER_STOP_MODE )
//	{
//		User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
//		User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
//	}
//	else if( User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE )	// �����ӵ�ʱ
	{
		// ���ݲ���ϵͳ��������
//		if( robot_state.shooter_id1_17mm_speed_limit == 15 )
//		{
//			if( shoot_data_t.bullet_speed > speed_limit )
//			{
//				shoot15_speed = 200*(shoot_data_t.bullet_speed - speed_limit);
//			}
			// ������������15m/s�Ļ�
//			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_15 - shoot15_speed;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_15 - shoot15_speed;
//		}
//		else if( robot_state.shooter_id1_17mm_speed_limit == 18 )
//		{
//			if( shoot_data_t.bullet_speed > speed_limit )
//			{
//				shoot18_speed = 200*(shoot_data_t.bullet_speed - speed_limit);
//			}
//			// ������������18m/s�Ļ�
//			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_18 - shoot18_speed;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_18 - shoot18_speed;
//		}
//		else if( robot_state.shooter_id1_17mm_speed_limit == 30 )
//		{
//			if( shoot_data_t.bullet_speed > speed_limit )
//			{
//				shoot30_speed = 200*(shoot_data_t.bullet_speed - speed_limit);
//			}
//			// ������������30m/s�Ļ�
//			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_30 - shoot30_speed;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_30 - shoot30_speed;
//		}
//		else
//		{
            User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_15;
            User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_15;
//			// ��������ǵĻ����Ǿ��ٶ�����Ϊ0������
//			User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
		//}
		
		#if EXTRA_3508_L_ROTATE_DIR
			//User_Motor_DataType->motor_extra_3508_l.speed_set = User_Motor_DataType->motor_extra_3508_l.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_l.speed_set = -User_Motor_DataType->motor_extra_3508_l.speed_set;
		#endif
		#if EXTRA_3508_R_ROTATE_DIR
			//User_Motor_DataType->motor_extra_3508_r.speed_set = User_Motor_DataType->motor_extra_3508_r.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_r.speed_set = -User_Motor_DataType->motor_extra_3508_r.speed_set;
		#endif
	}
}

void Motor_2006_Data_Processing( User_Motor_t *User_Motor_DataType )
{
	if( User_Motor_DataType->motor_mode == MOTOR_GET_READY_MODE || User_Motor_DataType->motor_mode == CLEAR_BULLET_PRE_MODE )
	{
		#if MOTOR_2006_ROTATE_DIR
			User_Motor_DataType->motor_2006.encoder_pos_all_set += MOTOR_2006_ENCODER_ONE_SHOOT;
			if( User_Motor_DataType->motor_2006.encoder_pos_all_set >= MOTOR_2006_ENCODER_A_ROUND )
			{
				User_Motor_DataType->motor_2006.encoder_pos_all_set -= MOTOR_2006_ENCODER_A_ROUND;
			}
		#else
			User_Motor_DataType->motor_2006.encoder_pos_all_set -= MOTOR_2006_ENCODER_ONE_SHOOT;
			if( User_Motor_DataType->motor_2006.encoder_pos_all_set < 0 )
			{
				User_Motor_DataType->motor_2006.encoder_pos_all_set += MOTOR_2006_ENCODER_A_ROUND;
			}
		#endif
		if( User_Motor_DataType->motor_mode == MOTOR_GET_READY_MODE )		User_Motor_DataType->motor_mode = MOTOR_SHOOT_MODE;
		else if( User_Motor_DataType->motor_mode == CLEAR_BULLET_PRE_MODE )	User_Motor_DataType->motor_mode = CLEAR_BULLET_STA_MODE;
	}
	
	User_Motor_DataType->motor_2006.encoder_pos_ecd = User_Motor_DataType->motor_2006.motor_2006_measure->ecd;
	// ��ת���
	if( User_Motor_DataType->motor_2006.speed_set != 0 )		// �������ת��ʱ��ſ�ʼ����ת
	{
		if( ((User_Motor_DataType->motor_2006.encoder_pos_ecd - User_Motor_DataType->motor_2006.encoder_pos_ecd_last) < MOTOR_2006_LOCKED_ROTOR_ERROR) && 
			((User_Motor_DataType->motor_2006.encoder_pos_ecd - User_Motor_DataType->motor_2006.encoder_pos_ecd_last) > -MOTOR_2006_LOCKED_ROTOR_ERROR) )
		{
			User_Motor_DataType->locked_rotor_time++;	// ��Ϊ�Ƕ�ת�ˣ���ʼ����
		}
		else
		{
			User_Motor_DataType->locked_rotor_time--;	// ����
		}
		if( User_Motor_DataType->motor_mode == MOTOR_LOCKED_ROTOR )
		{
			User_Motor_DataType->reversal_time++;
		}
		
		// ��λ
		if( User_Motor_DataType->locked_rotor_time > MOTOR_2006_LOCKED_ROTOR_MAX )
			User_Motor_DataType->locked_rotor_time = MOTOR_2006_LOCKED_ROTOR_MAX;
		else if( User_Motor_DataType->locked_rotor_time < MOTOR_2006_LOCKED_ROTOR_MIN )
			User_Motor_DataType->locked_rotor_time = MOTOR_2006_LOCKED_ROTOR_MIN;
		
		if( User_Motor_DataType->locked_rotor_time >= MOTOR_2006_LOCKED_ROTOR_SHORT )
		{
			// ȷ��Ϊ��ת��
			User_Motor_DataType->motor_mode = MOTOR_LOCKED_ROTOR;
		}
		else if( User_Motor_DataType->locked_rotor_time >= MOTOR_2006_LOCKED_ROTOR_LONG )
		{
			// ��ʱ���ת��û���ˣ��ϵ�ɣ�������ûд�������ǿ��Լ�����
			User_Motor_DataType->motor_mode = MOTOR_USER_STOP_MODE;
		}
		
		if( User_Motor_DataType->motor_mode == MOTOR_LOCKED_ROTOR )
		{
			if( User_Motor_DataType->reversal_time > MOTOR_2006_LOCKED_ROTOR_REVERSAL )
			{
				// ��Ϊ��תʱ��ﵽ�����¿�ʼ��ת
				User_Motor_DataType->motor_mode = MOTOR_SHOOT_MODE;
				User_Motor_DataType->reversal_time = 0;
			}
		}
	}
	
	// ������2006���λ�ü���
	if( User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE || User_Motor_DataType->motor_mode == CLEAR_BULLET_STA_MODE )
	{
		int temp;
		
		// ����ת�˼�Ȧ
		if( User_Motor_DataType->motor_2006.encoder_pos_ecd - User_Motor_DataType->motor_2006.encoder_pos_ecd_last >= MOTOR_USER_HALF_ENCODER )
		{
			User_Motor_DataType->motor_2006.encoder_pos_count--;
		}
		else if( User_Motor_DataType->motor_2006.encoder_pos_ecd - User_Motor_DataType->motor_2006.encoder_pos_ecd_last <= -MOTOR_USER_HALF_ENCODER )
		{
			User_Motor_DataType->motor_2006.encoder_pos_count++;
		}
		User_Motor_DataType->motor_2006.encoder_pos_ecd_last = User_Motor_DataType->motor_2006.encoder_pos_ecd;
		
		if( User_Motor_DataType->motor_2006.encoder_pos_count >= MOTOR_2006_ENCODER_COUNT_MAX )
		{
			User_Motor_DataType->motor_2006.encoder_pos_count = 0;
		}
		else if( User_Motor_DataType->motor_2006.encoder_pos_count < 0 )
		{
			User_Motor_DataType->motor_2006.encoder_pos_count = MOTOR_2006_ENCODER_COUNT_MAX - 1;
		}
		
		User_Motor_DataType->motor_2006.encoder_pos_all = User_Motor_DataType->motor_2006.encoder_pos_count*(MOTOR_USER_MAX_ENCODER+1) + User_Motor_DataType->motor_2006.encoder_pos_ecd;
		#if MOTOR_2006_ROTATE_DIR
			temp = User_Motor_DataType->motor_2006.encoder_pos_all - User_Motor_DataType->motor_2006.encoder_pos_all_set;
			if( temp >= MOTOR_2006_ENCODER_HALF_ROUND )	temp = temp - MOTOR_2006_ENCODER_A_ROUND;
		#else
			temp = User_Motor_DataType->motor_2006.encoder_pos_all_set - User_Motor_DataType->motor_2006.encoder_pos_all;
			if( temp >= MOTOR_2006_ENCODER_HALF_ROUND )	temp = temp - MOTOR_2006_ENCODER_A_ROUND;	
		#endif
		if( temp > MOTOR_2006_ENCODER_ROUND_ERROR )
		{
			if( User_Motor_DataType->user_motor_RC->mouse.press_l )
			{
				User_Motor_DataType->motor_mode = MOTOR_GET_READY_MODE;
			}
			else if( User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE )
			{
				User_Motor_DataType->motor_mode = MOTOR_DONE_MODE;
			}
			else if( User_Motor_DataType->motor_mode == CLEAR_BULLET_STA_MODE )
			{
				User_Motor_DataType->motor_mode = CLEAR_BULLET_PRE_MODE;
			}
		}
	}
	
	// ������2006����ٶȶ�ȡ
	User_Motor_DataType->motor_2006.speed = User_Motor_DataType->motor_2006.motor_2006_measure->speed_rpm;
	// ������2006����ٶ�����
	if( User_Motor_DataType->motor_mode == MOTOR_USER_STOP_MODE )
	{
		User_Motor_DataType->motor_2006.speed_set = 0;
	}
	else if( (User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE) || (User_Motor_DataType->motor_mode == MOTOR_GET_READY_MODE) )
	{
		#if MOTOR_2006_ROTATE_DIR
			User_Motor_DataType->motor_2006.speed_set = MOTOR_2006_QUICK_ROTATE_SPEED;
		#else
			User_Motor_DataType->motor_2006.speed_set = -MOTOR_2006_QUICK_ROTATE_SPEED;
		#endif
	}
	else if( User_Motor_DataType->motor_mode == MOTOR_LOCKED_ROTOR )
	{
		// ��ת����ʼ��ת
		#if MOTOR_2006_ROTATE_DIR
			User_Motor_DataType->motor_2006.speed_set = -MOTOR_2006_QUICK_ROTATE_SPEED;
		#else
			User_Motor_DataType->motor_2006.speed_set = MOTOR_2006_QUICK_ROTATE_SPEED;
		#endif
	}
	else if( User_Motor_DataType->motor_mode == MOTOR_DONE_MODE )
	{
		User_Motor_DataType->motor_2006.speed_set = 0;
	}
}

void Extra_3508_Speed_PID( User_Motor_t *User_Motor_PIDType )
{
	// ����Ħ����3508���PID����
	//uint8_t i;
	
	PID_calc( &User_Motor_PIDType->motor_extra_3508_l.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_l.speed, User_Motor_PIDType->motor_extra_3508_l.speed_set);
	User_Motor_PIDType->motor_extra_3508_l.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_l.motor_speed_pid.out;
	PID_calc( &User_Motor_PIDType->motor_extra_3508_r.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_r.speed, User_Motor_PIDType->motor_extra_3508_r.speed_set);
	User_Motor_PIDType->motor_extra_3508_r.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_r.motor_speed_pid.out;
}

void Motor_2006_PID_Set( User_Motor_t *User_Motor_PIDType )
{
	// һ��������2006���PID����
	PID_calc( &User_Motor_PIDType->motor_2006.motor_speed_pid, User_Motor_PIDType->motor_2006.speed, User_Motor_PIDType->motor_2006.speed_set);
	User_Motor_PIDType->motor_2006.given_current = (int16_t)User_Motor_PIDType->motor_2006.motor_speed_pid.out;
}

//void ExBoard_Angle_Init_Mode( CAN_HandleTypeDef *hcan, uint16_t addr )
//{
//	uint8_t buf[8] = { 0 };
//	buf[7] = MPU6050_ANGLE_INIT_MODE;
//	CAN_SEND_MESSAGE( hcan, buf, addr );
//}
//����ǵ����ŵĶ��
//void ExBoard_Servo_Control_Mode( CAN_HandleTypeDef *hcan, uint16_t addr, uint16_t angle1, uint16_t angle2 )
//{
//	uint8_t buf[8] = { 0 };
//	buf[7] = SERVO_ANGLE_SET_MODE;
//	buf[0] = angle1 >> 8;
//	buf[1] = angle1;
//	buf[2] = angle2 >> 8;
//	buf[3] = angle2;
//	CAN_SEND_MESSAGE( hcan, buf, addr );
//}

// ǿ�Ƹ�λ����
//void CAR_FORCE_RESET( void )
//{
//	__set_FAULTMASK(1);//�ر������ж�
//	NVIC_SystemReset();//��λ����
//}
