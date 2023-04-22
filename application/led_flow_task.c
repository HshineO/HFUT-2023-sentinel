/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGB灯效。
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
////蓝 -> 绿(灭) -> 红 -> 蓝(灭) -> 绿 -> 红(灭) -> 蓝
//uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};


/**
  * @brief          led RGB任务
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

// 裁判系统数据
extern ext_game_robot_state_t robot_state;

void led_RGB_flow_task(void const *argument)
{
	vTaskDelay(50);
	Motor_User_Init(&user_motor);

  while(1)
  {
	  //if ( robot_state.remain_HP )	// 还活着
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
	  //else	// 不好意思您已经壮烈牺牲
	  //{
		  
	  //}
		
//	  usb_printf("%6d,%6d.", user_motor.motor_extra_3508_l.extra_3508_measure->speed_rpm, user_motor.motor_extra_3508_r.extra_3508_measure->speed_rpm );
		vTaskDelay(2);	
	}
}
// 额外的3508电机控制代码
void Motor_User_Init( User_Motor_t *User_Motor_InitType )
{
	const static fp32 extra_3508_speed_pid[3] = { EXTRA_3508_SPEED_KP, EXTRA_3508_SPEED_KI, EXTRA_3508_SPEED_KD };
	const static fp32 motor_2006_speed_pid[3] = { MOTOR_2006_SPEED_KP, MOTOR_2006_SPEED_KI, MOTOR_2006_SPEED_KD };
	
	// 模式初始化
	User_Motor_InitType->motor_mode = MOTOR_USER_STOP_MODE;
	
	// 遥控指针初始化
	User_Motor_InitType->user_motor_RC = get_remote_control_point();
	
	// 2006电机参数初始化
	User_Motor_InitType->motor_2006.encoder_pos_count = 1;
	User_Motor_InitType->motor_2006.encoder_pos_all = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_2006.encoder_pos_all_set = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_2006.encoder_pos_ecd = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_2006.encoder_pos_ecd_last = User_Motor_InitType->motor_2006.motor_2006_measure->ecd;
	
	// 3508电机参数初始化
	User_Motor_InitType->motor_extra_3508_l.given_current = 0;
	User_Motor_InitType->motor_extra_3508_l.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_l.speed = 0;
	User_Motor_InitType->motor_extra_3508_r.given_current = 0;
	User_Motor_InitType->motor_extra_3508_r.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_r.speed = 0;
	
	// 3508 PID 初始化
	User_Motor_InitType->motor_extra_3508_l.extra_3508_measure = get_shoot_motor_point(1);
	PID_init( &User_Motor_InitType->motor_extra_3508_l.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	User_Motor_InitType->motor_extra_3508_r.extra_3508_measure = get_shoot_motor_point(2);
	PID_init( &User_Motor_InitType->motor_extra_3508_r.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	// 2006 PID 初始化
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
			if( (User_Motor_ModeTypedef->motor_mode==MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode==MOTOR_DONE_MODE) )//点射注释这里
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
	// 遥控设置电机模式
//	if( switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]) )	
//	{
//		shoot_count++;
//		
//		if( shoot_count == 10 )		User_Motor_ModeTypedef->motor_mode = MOTOR_GET_READY_MODE;	// 发射弹丸一次
////		if( shoot_count == 8000 )	shoot_count = 8000, User_Motor_ModeTypedef->motor_mode = CLEAR_BULLET_PRE_MODE;	// 准备清空所有子弹
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
	// else if( switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]) )	// 使用键盘控制模式
	// {
	// 	// 键盘设置电机模式
	// 	if( (User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_SHIFT_CTRL) == KEY_PRESSED_OFFSET_SHIFT_CTRL )	// shift和ctrl都按下
	// 	{
	// 		if( User_Motor_ModeTypedef->user_motor_RC->key.v & CONTROL_GUARD_TO_FLEE )	// 控制哨兵逃跑
	// 		{
	// 		}
	// 		else if( User_Motor_ModeTypedef->user_motor_RC->key.v & CONTROL_GUARD_TO_ATTACK )		// 控制哨兵进攻
	// 		{
	// 		}
	// 	}
	// 	else if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )	// 只有shift按下
	// 	{
			
	// 	}
	// 	else if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_CTRL )	// 只有ctrl都按下
	// 	{
			
	// 	}
	// 	// 鼠标检测
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
	// 	  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_CTRL )	// ctrl按下，强行复位按钮
	// 	  {
	// 		  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )	// shift按下，强行复位按钮
	// 		  {
	// 			  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_A )	// A按下，强行复位按钮
	// 			  {
	// 				  if( User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_D )	// D按下，强行复位按钮
	// 				  {
	// 					  CAR_FORCE_RESET();
	// 				  }
	// 			  }
	// 		  }
	// 	  }
	// 	}
	// }
	// if ( switch_is_down(User_Motor_ModeTypedef->user_motor_RC->rc.s[0]) )	// 所有电机停转，除拨弹轮
	// {
	// 	User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
	// }
}
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType )
{
	static uint16_t shoot15_speed = 0, shoot18_speed = 0, shoot30_speed = 0;
	//fp32 speed_limit = robot_state.shooter_id1_17mm_speed_limit;
	
	// 两个摩擦轮3508电机数据处理
	User_Motor_DataType->motor_extra_3508_l.speed = User_Motor_DataType->motor_extra_3508_l.extra_3508_measure->speed_rpm;
	User_Motor_DataType->motor_extra_3508_r.speed = User_Motor_DataType->motor_extra_3508_r.extra_3508_measure->speed_rpm;
	
	
	
//	if( User_Motor_DataType->motor_mode == MOTOR_USER_STOP_MODE )
//	{
//		User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
//		User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
//	}
//	else if( User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE )	// 发射子弹时
	{
		// 根据裁判系统设置射速
//		if( robot_state.shooter_id1_17mm_speed_limit == 15 )
//		{
//			if( shoot_data_t.bullet_speed > speed_limit )
//			{
//				shoot15_speed = 200*(shoot_data_t.bullet_speed - speed_limit);
//			}
			// 如果最大射速是15m/s的话
//			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_15 - shoot15_speed;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_15 - shoot15_speed;
//		}
//		else if( robot_state.shooter_id1_17mm_speed_limit == 18 )
//		{
//			if( shoot_data_t.bullet_speed > speed_limit )
//			{
//				shoot18_speed = 200*(shoot_data_t.bullet_speed - speed_limit);
//			}
//			// 如果最大射速是18m/s的话
//			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_18 - shoot18_speed;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_18 - shoot18_speed;
//		}
//		else if( robot_state.shooter_id1_17mm_speed_limit == 30 )
//		{
//			if( shoot_data_t.bullet_speed > speed_limit )
//			{
//				shoot30_speed = 200*(shoot_data_t.bullet_speed - speed_limit);
//			}
//			// 如果最大射速是30m/s的话
//			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_30 - shoot30_speed;
//			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_30 - shoot30_speed;
//		}
//		else
//		{
            User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_15;
            User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_15;
//			// 如果都不是的话，那就速度设置为0来报错
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
	// 堵转检测
	if( User_Motor_DataType->motor_2006.speed_set != 0 )		// 当电机旋转的时候才开始检测堵转
	{
		if( ((User_Motor_DataType->motor_2006.encoder_pos_ecd - User_Motor_DataType->motor_2006.encoder_pos_ecd_last) < MOTOR_2006_LOCKED_ROTOR_ERROR) && 
			((User_Motor_DataType->motor_2006.encoder_pos_ecd - User_Motor_DataType->motor_2006.encoder_pos_ecd_last) > -MOTOR_2006_LOCKED_ROTOR_ERROR) )
		{
			User_Motor_DataType->locked_rotor_time++;	// 认为是堵转了，开始计数
		}
		else
		{
			User_Motor_DataType->locked_rotor_time--;	// 防抖
		}
		if( User_Motor_DataType->motor_mode == MOTOR_LOCKED_ROTOR )
		{
			User_Motor_DataType->reversal_time++;
		}
		
		// 限位
		if( User_Motor_DataType->locked_rotor_time > MOTOR_2006_LOCKED_ROTOR_MAX )
			User_Motor_DataType->locked_rotor_time = MOTOR_2006_LOCKED_ROTOR_MAX;
		else if( User_Motor_DataType->locked_rotor_time < MOTOR_2006_LOCKED_ROTOR_MIN )
			User_Motor_DataType->locked_rotor_time = MOTOR_2006_LOCKED_ROTOR_MIN;
		
		if( User_Motor_DataType->locked_rotor_time >= MOTOR_2006_LOCKED_ROTOR_SHORT )
		{
			// 确认为堵转了
			User_Motor_DataType->motor_mode = MOTOR_LOCKED_ROTOR;
		}
		else if( User_Motor_DataType->locked_rotor_time >= MOTOR_2006_LOCKED_ROTOR_LONG )
		{
			// 长时间堵转，没救了，断电吧，但程序没写死，还是可以继续动
			User_Motor_DataType->motor_mode = MOTOR_USER_STOP_MODE;
		}
		
		if( User_Motor_DataType->motor_mode == MOTOR_LOCKED_ROTOR )
		{
			if( User_Motor_DataType->reversal_time > MOTOR_2006_LOCKED_ROTOR_REVERSAL )
			{
				// 认为反转时间达到，重新开始正转
				User_Motor_DataType->motor_mode = MOTOR_SHOOT_MODE;
				User_Motor_DataType->reversal_time = 0;
			}
		}
	}
	
	// 拨弹轮2006电机位置计算
	if( User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE || User_Motor_DataType->motor_mode == CLEAR_BULLET_STA_MODE )
	{
		int temp;
		
		// 计算转了几圈
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
	
	// 拨弹轮2006电机速度读取
	User_Motor_DataType->motor_2006.speed = User_Motor_DataType->motor_2006.motor_2006_measure->speed_rpm;
	// 拨弹轮2006电机速度设置
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
		// 堵转，开始反转
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
	// 两个摩擦轮3508电机PID计算
	//uint8_t i;
	
	PID_calc( &User_Motor_PIDType->motor_extra_3508_l.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_l.speed, User_Motor_PIDType->motor_extra_3508_l.speed_set);
	User_Motor_PIDType->motor_extra_3508_l.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_l.motor_speed_pid.out;
	PID_calc( &User_Motor_PIDType->motor_extra_3508_r.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_r.speed, User_Motor_PIDType->motor_extra_3508_r.speed_set);
	User_Motor_PIDType->motor_extra_3508_r.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_r.motor_speed_pid.out;
}

void Motor_2006_PID_Set( User_Motor_t *User_Motor_PIDType )
{
	// 一个拨弹轮2006电机PID计算
	PID_calc( &User_Motor_PIDType->motor_2006.motor_speed_pid, User_Motor_PIDType->motor_2006.speed, User_Motor_PIDType->motor_2006.speed_set);
	User_Motor_PIDType->motor_2006.given_current = (int16_t)User_Motor_PIDType->motor_2006.motor_speed_pid.out;
}

//void ExBoard_Angle_Init_Mode( CAN_HandleTypeDef *hcan, uint16_t addr )
//{
//	uint8_t buf[8] = { 0 };
//	buf[7] = MPU6050_ANGLE_INIT_MODE;
//	CAN_SEND_MESSAGE( hcan, buf, addr );
//}
//这个是弹仓门的舵机
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

// 强制复位函数
//void CAR_FORCE_RESET( void )
//{
//	__set_FAULTMASK(1);//关闭所有中断
//	NVIC_SystemReset();//复位函数
//}
