/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
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

#include "shoot_task.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#include "USB_task.h"

#include "ANO_DT.h"

#include "remote_control.h"
#include "FreeRTOS.h"




User_Motor_t user_motor;

static fp32 Extra_3508_SetCurrent_high_L = 0,Extra_3508_SetCurrent_low_L = 0, Extra_3508_SetCurrent_high_R = 0,Extra_3508_SetCurrent_low_R = 0, Motor_2006_L_Motor_Set_Current = 0,Motor_2006_R_Motor_Set_Current = 0;
volatile extern int16_t Yaw_Can_Set_Current, Pitch_Can_Set_Current;
extern uint16_t reset_count;
extern ext_robot_hurt_t robot_hurt_t;
extern gimbal_control_t gimbal_control;
extern RecievePacket_t recievePacket;
extern RC_ctrl_t rc_ctrl;


static uint16_t Locked_Time_Tick_L= 0;
static uint16_t Locked_Time_Tick_R= 0;
extern osTimerId Anti_Locked_L_TimerHandle;
extern osTimerId Anti_Locked_R_TimerHandle;


fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

// ����ϵͳ����
extern ext_game_robot_state_t robot_state;

void shoot_task(void const * argument)
{
	vTaskDelay(50);
	Motor_User_Init(&user_motor);

  	while(1)
  	{
	  //if ( robot_state.remain_HP )	// ������
	  {
		Motor_User_Mode_Set(&user_motor); //״̬������
		  
		Extra_3508_Data_Processing(&user_motor);//Ħ����3508���ݴ���
		  
		Extra_3508_Speed_PID(&user_motor);//Ħ����3508PID����
		  
		Motor_2006_Data_Processing(&user_motor); //2006�����������&����ת
		  
		Motor_2006_PID_Set(&user_motor);//�������PID����

		
		  //����3508�ĵ���
		Extra_3508_SetCurrent_high_L = user_motor.motor_extra_3508_high_l.given_current;
		Extra_3508_SetCurrent_low_L = user_motor.motor_extra_3508_low_l.given_current;
		  
		Extra_3508_SetCurrent_high_R = user_motor.motor_extra_3508_high_r.given_current;
		     Extra_3508_SetCurrent_low_R = user_motor.motor_extra_3508_low_r.given_current;
		  
		  //�������������̵ĵ���
		Motor_2006_L_Motor_Set_Current = user_motor.motor_l_2006.given_current;
		Motor_2006_R_Motor_Set_Current = user_motor.motor_r_2006.given_current;


		CAN_CMD_Extra3508( Extra_3508_SetCurrent_high_L, Extra_3508_SetCurrent_low_L,Motor_2006_L_Motor_Set_Current, Extra_3508_SetCurrent_high_R );
		CAN_cmd_chassis_shoot(Extra_3508_SetCurrent_low_R,Motor_2006_R_Motor_Set_Current,0,0);

	  }
	  //else	// ������˼���Ѿ�׳������
	  //{  
	  //}
		
//	  usb_printf("%6d,%6d.", user_motor.motor_extra_3508_l.extra_3508_measure->speed_rpm, user_motor.motor_extra_3508_r.extra_3508_measure->speed_rpm );
//    ANODT_SendF1((user_motor.motor_r_2006.encoder_pos_ecd - user_motor.motor_r_2006.encoder_pos_ecd_last),(user_motor.motor_l_2006.encoder_pos_ecd - user_motor.motor_l_2006.encoder_pos_ecd_last),
//				 user_motor.motor_r_2006.encoder_pos_ecd, user_motor.motor_l_2006.encoder_pos_ecd);
	vTaskDelay(2);	
	}

}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);



shoot_control_t shoot_control;          //�������


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //��ʼ��PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    //��������
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������


    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            //���ò����ֵĲ����ٶ�,��������ת��ת����
            shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            shoot_control.trigger_speed_set = 0.0f;
            shoot_control.speed_set = 0.0f;
        }
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //���ò����ֵ��ٶ�
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //���ò����ֵĲ����ٶ�,��������ת��ת����
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        shoot_laser_on(); //���⿪��
        //���㲦���ֵ��PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);
    return shoot_control.given_current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time++;
            if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    


    if(shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        //��곤��һֱ�������״̬ ��������
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }

    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //΢������
    shoot_control.key = BUTTEN_TRIG_PIN;
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
    static uint16_t up_time = 0;
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;
    }

    if (up_time > 0)
    {
        shoot_control.fric1_ramp.max_value = FRIC_UP;
        shoot_control.fric2_ramp.max_value = FRIC_UP;
        up_time--;
    }
    else
    {
        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    }


}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {

        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //����Ƕ��ж�
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}


//���Ǳ�д��˫ǹ���뿩~
void Motor_User_Init( User_Motor_t *User_Motor_InitType )
{
	const static fp32 extra_3508_speed_pid[3] = { EXTRA_3508_SPEED_KP, EXTRA_3508_SPEED_KI, EXTRA_3508_SPEED_KD };
	const static fp32 motor_2006_speed_pid[3] = { MOTOR_2006_SPEED_KP, MOTOR_2006_SPEED_KI, MOTOR_2006_SPEED_KD };
	
	// ģʽ��ʼ��
	User_Motor_InitType->motor_mode_l = MOTOR_USER_STOP_MODE;
	User_Motor_InitType->motor_mode_r = MOTOR_USER_STOP_MODE;
	
	// ң��ָ���ʼ��
	User_Motor_InitType->user_motor_RC = get_remote_control_point();
	
	// 2006���������ʼ��
	User_Motor_InitType->motor_l_2006.encoder_pos_count = 1;
	User_Motor_InitType->motor_l_2006.encoder_pos_all = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_l_2006.encoder_pos_all_set = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_l_2006.encoder_pos_ecd = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_l_2006.encoder_pos_ecd_last = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	
	User_Motor_InitType->motor_r_2006.encoder_pos_count = 1;
	User_Motor_InitType->motor_r_2006.encoder_pos_all = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_r_2006.encoder_pos_all_set = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_r_2006.encoder_pos_ecd = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_InitType->motor_r_2006.encoder_pos_ecd_last = User_Motor_InitType->motor_l_2006.motor_2006_measure->ecd;
	
	// 3508���������ʼ��
	User_Motor_InitType->motor_extra_3508_high_l.given_current = 0;
	User_Motor_InitType->motor_extra_3508_high_l.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_high_l.speed = 0;
	User_Motor_InitType->motor_extra_3508_low_l.given_current = 0;
	User_Motor_InitType->motor_extra_3508_low_l.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_low_l.speed = 0;
	User_Motor_InitType->motor_extra_3508_high_r.given_current = 0;
	User_Motor_InitType->motor_extra_3508_high_r.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_high_r.speed = 0;
	User_Motor_InitType->motor_extra_3508_low_r.given_current = 0;
	User_Motor_InitType->motor_extra_3508_low_r.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_low_r.speed = 0;
	
	// 3508 PID ��ʼ��
	User_Motor_InitType->motor_extra_3508_high_l.extra_3508_measure = get_shoot_motor_point(0);
	PID_init( &User_Motor_InitType->motor_extra_3508_high_l.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	User_Motor_InitType->motor_extra_3508_low_l.extra_3508_measure = get_shoot_motor_point(1);
	PID_init( &User_Motor_InitType->motor_extra_3508_low_l.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	User_Motor_InitType->motor_extra_3508_high_r.extra_3508_measure = get_shoot_motor_point(3);
	PID_init( &User_Motor_InitType->motor_extra_3508_high_r.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	User_Motor_InitType->motor_extra_3508_low_r.extra_3508_measure = get_shoot_motor_point(4);
	PID_init( &User_Motor_InitType->motor_extra_3508_low_r.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT );
	// 2006 PID ��ʼ��
	User_Motor_InitType->motor_l_2006.motor_2006_measure = get_shoot_motor_point(2);
	PID_init( &User_Motor_InitType->motor_l_2006.motor_speed_pid, PID_POSITION, motor_2006_speed_pid, MOTOR_2006_SPEED_PID_MAXOUT, MOTOR_2006_SPEED_PID_MAXIOUT );
	User_Motor_InitType->motor_r_2006.motor_2006_measure = get_shoot_motor_point(5);
	PID_init( &User_Motor_InitType->motor_r_2006.motor_speed_pid, PID_POSITION, motor_2006_speed_pid, MOTOR_2006_SPEED_PID_MAXOUT, MOTOR_2006_SPEED_PID_MAXIOUT );
	
	User_Motor_InitType->locked_rotor_time_l = 0;
	User_Motor_InitType->reversal_time_l = 0;
	User_Motor_InitType->locked_rotor_time_r = 0;
	User_Motor_InitType->reversal_time_r = 0;
}
void Motor_User_Mode_Set( User_Motor_t *User_Motor_ModeTypedef )
{
	static uint16_t shoot_count = 0;
	
	if(gimbal_control.gimbal_control_mode == GIMBAL_RC)
	{
		if(switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[1]))	
		{
			if((User_Motor_ModeTypedef->motor_mode_l == MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode_l == MOTOR_DONE_MODE) )//����ע������
			{
				User_Motor_ModeTypedef->motor_mode_l = MOTOR_GET_READY_MODE;
			}
			if( (User_Motor_ModeTypedef->motor_mode_r == MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode_r == MOTOR_DONE_MODE) )//����ע������
			{
				User_Motor_ModeTypedef->motor_mode_r = MOTOR_GET_READY_MODE;
			}
		}
		else if( switch_is_mid(User_Motor_ModeTypedef->user_motor_RC->rc.s[1] ))
		{
			User_Motor_ModeTypedef->motor_mode_l = MOTOR_USER_STOP_MODE;
			User_Motor_ModeTypedef->motor_mode_r = MOTOR_USER_STOP_MODE;
		}
		else
		{
			shoot_count = 0;
		}
	}
	else if(gimbal_control.gimbal_control_mode == GIMBAL_AUTO)
	{
		if(recievePacket.suggest_fire == 1)
		{
			if( (User_Motor_ModeTypedef->motor_mode_l==MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode_l==MOTOR_DONE_MODE) )//����ע������
			{
				User_Motor_ModeTypedef->motor_mode_l = MOTOR_GET_READY_MODE;
			}
			if( (User_Motor_ModeTypedef->motor_mode_r==MOTOR_USER_STOP_MODE) || (User_Motor_ModeTypedef->motor_mode_r==MOTOR_DONE_MODE) )//����ע������
			{
				User_Motor_ModeTypedef->motor_mode_r = MOTOR_GET_READY_MODE;
			}
		}
		else if(recievePacket.suggest_fire == 0)
		{
			User_Motor_ModeTypedef->motor_mode_l = MOTOR_USER_STOP_MODE;
			User_Motor_ModeTypedef->motor_mode_r = MOTOR_USER_STOP_MODE;
		}
		else
		{
			shoot_count = 0;
		}
	}
}
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType )
{
	static uint16_t shoot15_speed = 0, shoot18_speed = 0, shoot30_speed = 0;

	// ����Ħ����3508������ݴ���
	User_Motor_DataType->motor_extra_3508_high_l.speed = User_Motor_DataType->motor_extra_3508_high_l.extra_3508_measure->speed_rpm;
	User_Motor_DataType->motor_extra_3508_low_l.speed = User_Motor_DataType->motor_extra_3508_low_l.extra_3508_measure->speed_rpm;
 	User_Motor_DataType->motor_extra_3508_high_r.speed = User_Motor_DataType->motor_extra_3508_high_r.extra_3508_measure->speed_rpm;
	User_Motor_DataType->motor_extra_3508_low_r.speed = User_Motor_DataType->motor_extra_3508_low_r.extra_3508_measure->speed_rpm;
	
	
	User_Motor_DataType->motor_extra_3508_high_l.speed_set = EXTRA_3508_ROTATE_SPEED_30;
	User_Motor_DataType->motor_extra_3508_low_l.speed_set  = EXTRA_3508_ROTATE_SPEED_30;
	User_Motor_DataType->motor_extra_3508_high_r.speed_set = EXTRA_3508_ROTATE_SPEED_30;
	User_Motor_DataType->motor_extra_3508_low_r.speed_set  = EXTRA_3508_ROTATE_SPEED_30;

	User_Motor_DataType->motor_extra_3508_high_l.speed_set =  User_Motor_DataType->motor_extra_3508_high_l.speed_set;
	User_Motor_DataType->motor_extra_3508_high_r.speed_set = -User_Motor_DataType->motor_extra_3508_high_r.speed_set;

	User_Motor_DataType->motor_extra_3508_low_l.speed_set  = -User_Motor_DataType->motor_extra_3508_low_l.speed_set;
	User_Motor_DataType->motor_extra_3508_low_r.speed_set  =  User_Motor_DataType->motor_extra_3508_low_r.speed_set;

}

void Motor_2006_Data_Processing( User_Motor_t *User_Motor_DataType )
{
	if( User_Motor_DataType->motor_mode_l == MOTOR_GET_READY_MODE || User_Motor_DataType->motor_mode_l == CLEAR_BULLET_PRE_MODE )
	{
		User_Motor_DataType->motor_l_2006.encoder_pos_all_set -= MOTOR_2006_ENCODER_ONE_SHOOT;
		if( User_Motor_DataType->motor_l_2006.encoder_pos_all_set < 0 )
		{
			User_Motor_DataType->motor_l_2006.encoder_pos_all_set += MOTOR_2006_ENCODER_A_ROUND;
		}

		if( User_Motor_DataType->motor_mode_l == MOTOR_GET_READY_MODE )		User_Motor_DataType->motor_mode_l = MOTOR_SHOOT_MODE;
		else if( User_Motor_DataType->motor_mode_l == CLEAR_BULLET_PRE_MODE )	User_Motor_DataType->motor_mode_l = CLEAR_BULLET_STA_MODE;
	}

	if( User_Motor_DataType->motor_mode_r == MOTOR_GET_READY_MODE || User_Motor_DataType->motor_mode_r == CLEAR_BULLET_PRE_MODE )
	{
		User_Motor_DataType->motor_r_2006.encoder_pos_all_set -= MOTOR_2006_ENCODER_ONE_SHOOT;
		if( User_Motor_DataType->motor_r_2006.encoder_pos_all_set < 0 )
		{
			User_Motor_DataType->motor_r_2006.encoder_pos_all_set += MOTOR_2006_ENCODER_A_ROUND;
		}

		if( User_Motor_DataType->motor_mode_r == MOTOR_GET_READY_MODE )		User_Motor_DataType->motor_mode_r = MOTOR_SHOOT_MODE;
		else if( User_Motor_DataType->motor_mode_r == CLEAR_BULLET_PRE_MODE )	User_Motor_DataType->motor_mode_r = CLEAR_BULLET_STA_MODE;
		
	}
	//������2006���״̬����
	User_Motor_DataType->motor_l_2006.encoder_pos_ecd = User_Motor_DataType->motor_l_2006.motor_2006_measure->ecd;
	User_Motor_DataType->motor_r_2006.encoder_pos_ecd = User_Motor_DataType->motor_r_2006.motor_2006_measure->ecd;
	#if 0
	// ��ת���  ���Һ��󣬷ֿ��ж�
	if( User_Motor_DataType->motor_r_2006.speed_set != 0 )		// �������ת��ʱ��ſ�ʼ����ת
	{
		if( ((User_Motor_DataType->motor_r_2006.encoder_pos_ecd - User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last) < MOTOR_2006_LOCKED_ROTOR_ERROR) && 
			((User_Motor_DataType->motor_r_2006.encoder_pos_ecd - User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last) > -MOTOR_2006_LOCKED_ROTOR_ERROR) )
		{
			xTimerStart( Anti_Locked_R_TimerHandle , Block_Time_Tick );
		}
		if( Locked_Time_Tick_R >= MOTOR_2006_LOCKED_ROTOR_SHORT_Tick )
		{
			// ȷ��Ϊ��ת��
			User_Motor_DataType->motor_mode_r = MOTOR_LOCKED_ROTOR;
		}
		
			else if( Locked_Time_Tick_R >= MOTOR_2006_LOCKED_ROTOR_LONG_Tick )
			{ 
				// ��ʱ���ת��û���ˣ��ϵ�ɣ�������ûд�������ǿ��Լ�����
				User_Motor_DataType->motor_mode_r = MOTOR_USER_STOP_MODE;
			}
			
		if( User_Motor_DataType->motor_mode_r == MOTOR_LOCKED_ROTOR )
		{
			User_Motor_DataType->reversal_time_r++;
		}
		
		
		if( User_Motor_DataType->motor_mode_r == MOTOR_LOCKED_ROTOR )
		{
			if( User_Motor_DataType->reversal_time_r > MOTOR_2006_LOCKED_ROTOR_REVERSAL )
			{
				// ��Ϊ��תʱ��ﵽ�����¿�ʼ��ת
				User_Motor_DataType->motor_mode_r = MOTOR_SHOOT_MODE;
				User_Motor_DataType->reversal_time_r = 0;
				xTimerStop( Anti_Locked_R_TimerHandle );
			}
		}
	}
	
	if( User_Motor_DataType->motor_l_2006.speed_set != 0 )		// �������ת��ʱ��ſ�ʼ����ת
	{
		if( ((User_Motor_DataType->motor_l_2006.encoder_pos_ecd - User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last) < MOTOR_2006_LOCKED_ROTOR_ERROR) && 
			((User_Motor_DataType->motor_l_2006.encoder_pos_ecd - User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last) > -MOTOR_2006_LOCKED_ROTOR_ERROR) )
		{
			xTimerStart( Anti_Locked_L_TimerHandle , Block_Time_Tick );
		}
		
		if( User_Motor_DataType->motor_mode_l == MOTOR_LOCKED_ROTOR )
		{
			User_Motor_DataType->reversal_time_l++;
		}
		
		if( User_Motor_DataType->locked_rotor_time_l >= MOTOR_2006_LOCKED_ROTOR_LONG_Tick  )
		{
			// ��ʱ���ת��û���ˣ��ϵ�ɣ�������ûд�������ǿ��Լ�����
			User_Motor_DataType->motor_mode_l = MOTOR_USER_STOP_MODE;
			
		}
		else if( User_Motor_DataType->locked_rotor_time_l >=  MOTOR_2006_LOCKED_ROTOR_SHORT )
		{
			// ȷ��Ϊ��ת��
			User_Motor_DataType->motor_mode_l = MOTOR_LOCKED_ROTOR;
		}
		
		if( User_Motor_DataType->motor_mode_l == MOTOR_LOCKED_ROTOR )
		{
			if( User_Motor_DataType->reversal_time_l > MOTOR_2006_LOCKED_ROTOR_REVERSAL )
			{
				// ��Ϊ��תʱ��ﵽ�����¿�ʼ��ת
				User_Motor_DataType->motor_mode_l = MOTOR_SHOOT_MODE;
				User_Motor_DataType->reversal_time_l = 0;
			}
		}
	}	
	#elseif
	
	if( User_Motor_DataType->motor_r_2006.speed_set != 0 )		// �������ת��ʱ��ſ�ʼ����ת
	{
		if( ((User_Motor_DataType->motor_r_2006.encoder_pos_ecd - User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last) < MOTOR_2006_LOCKED_ROTOR_ERROR) && 
			((User_Motor_DataType->motor_r_2006.encoder_pos_ecd - User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last) > -MOTOR_2006_LOCKED_ROTOR_ERROR) )
		{
			User_Motor_DataType->locked_rotor_time_r++;	// ��Ϊ�Ƕ�ת�ˣ���ʼ����
			//xTimerStart( Anti_Locked_R_TimerHandle , Block_Time_Tick );
		}
		else
		{
			User_Motor_DataType->locked_rotor_time_r--;	// ����
		}
		if( User_Motor_DataType->motor_mode_r == MOTOR_LOCKED_ROTOR )
		{
			User_Motor_DataType->reversal_time_r++;
		}
		
		// ��λ
		if( User_Motor_DataType->locked_rotor_time_r > MOTOR_2006_LOCKED_ROTOR_MAX )
			User_Motor_DataType->locked_rotor_time_r = MOTOR_2006_LOCKED_ROTOR_MAX;
		else if( User_Motor_DataType->locked_rotor_time_r < MOTOR_2006_LOCKED_ROTOR_MIN )
			User_Motor_DataType->locked_rotor_time_r = MOTOR_2006_LOCKED_ROTOR_MIN;
		
		if( User_Motor_DataType->locked_rotor_time_r >= MOTOR_2006_LOCKED_ROTOR_SHORT )
		{
			// ȷ��Ϊ��ת��
			User_Motor_DataType->motor_mode_r = MOTOR_LOCKED_ROTOR;
		}
		else if( User_Motor_DataType->locked_rotor_time_r >= MOTOR_2006_LOCKED_ROTOR_LONG )
		{
			// ��ʱ���ת��û���ˣ��ϵ�ɣ�������ûд�������ǿ��Լ�����
			User_Motor_DataType->motor_mode_r = MOTOR_USER_STOP_MODE;
		}
		
		if( User_Motor_DataType->motor_mode_r == MOTOR_LOCKED_ROTOR )
		{
			if( User_Motor_DataType->reversal_time_r > MOTOR_2006_LOCKED_ROTOR_REVERSAL )
			{
				// ��Ϊ��תʱ��ﵽ�����¿�ʼ��ת
				User_Motor_DataType->motor_mode_r = MOTOR_SHOOT_MODE;
				User_Motor_DataType->reversal_time_r = 0;
			}
		}
	}
	
	if( User_Motor_DataType->motor_l_2006.speed_set != 0 )		// �������ת��ʱ��ſ�ʼ����ת
	{
		if( ((User_Motor_DataType->motor_l_2006.encoder_pos_ecd - User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last) < MOTOR_2006_LOCKED_ROTOR_ERROR) && 
			((User_Motor_DataType->motor_l_2006.encoder_pos_ecd - User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last) > -MOTOR_2006_LOCKED_ROTOR_ERROR) )
		{
			User_Motor_DataType->locked_rotor_time_l++;	// ��Ϊ�Ƕ�ת�ˣ���ʼ����
			//xTimerStart( Anti_Locked_L_TimerHandle , Block_Time_Tick );
		}
		else
		{
			User_Motor_DataType->locked_rotor_time_l--;	// ����
		}
		if( User_Motor_DataType->motor_mode_l == MOTOR_LOCKED_ROTOR )
		{
			User_Motor_DataType->reversal_time_l++;
		}
		
		// ��λ
		if( User_Motor_DataType->locked_rotor_time_l > MOTOR_2006_LOCKED_ROTOR_MAX )
			User_Motor_DataType->locked_rotor_time_l = MOTOR_2006_LOCKED_ROTOR_MAX;
		else if( User_Motor_DataType->locked_rotor_time_l < MOTOR_2006_LOCKED_ROTOR_MIN )
			User_Motor_DataType->locked_rotor_time_l = MOTOR_2006_LOCKED_ROTOR_MIN;
		
		if( User_Motor_DataType->locked_rotor_time_l >= MOTOR_2006_LOCKED_ROTOR_SHORT )
		{
			// ȷ��Ϊ��ת��
			User_Motor_DataType->motor_mode_l = MOTOR_LOCKED_ROTOR;
		}
		else if( User_Motor_DataType->locked_rotor_time_l >= MOTOR_2006_LOCKED_ROTOR_LONG )
		{
			// ��ʱ���ת��û���ˣ��ϵ�ɣ�������ûд�������ǿ��Լ�����
			User_Motor_DataType->motor_mode_l = MOTOR_USER_STOP_MODE;
		}
		
		if( User_Motor_DataType->motor_mode_l == MOTOR_LOCKED_ROTOR )
		{
			if( User_Motor_DataType->reversal_time_l > MOTOR_2006_LOCKED_ROTOR_REVERSAL )
			{
				// ��Ϊ��תʱ��ﵽ�����¿�ʼ��ת
				User_Motor_DataType->motor_mode_l = MOTOR_SHOOT_MODE;
				User_Motor_DataType->reversal_time_l = 0;
			}
		}
	}
	#endif
	// ������2006���λ�ü���  ���Һ��󣬷ֿ��ж�
	if( User_Motor_DataType->motor_mode_l == MOTOR_SHOOT_MODE || User_Motor_DataType->motor_mode_l == CLEAR_BULLET_STA_MODE )
	{
		int temp_l;
		
		// ����ת�˼�Ȧ
		if( User_Motor_DataType->motor_l_2006.encoder_pos_ecd - User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last >= MOTOR_USER_HALF_ENCODER )
		{
			User_Motor_DataType->motor_l_2006.encoder_pos_count--;
		}
		else if( User_Motor_DataType->motor_l_2006.encoder_pos_ecd - User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last <= -MOTOR_USER_HALF_ENCODER )
		{
			User_Motor_DataType->motor_l_2006.encoder_pos_count++;
		}
		
		User_Motor_DataType->motor_l_2006.encoder_pos_ecd_last = User_Motor_DataType->motor_l_2006.encoder_pos_ecd;
		
		if( User_Motor_DataType->motor_l_2006.encoder_pos_count >= MOTOR_2006_ENCODER_COUNT_MAX )
		{
			User_Motor_DataType->motor_l_2006.encoder_pos_count = 0;
		}
		else if( User_Motor_DataType->motor_l_2006.encoder_pos_count < 0 )
		{
			User_Motor_DataType->motor_l_2006.encoder_pos_count = MOTOR_2006_ENCODER_COUNT_MAX - 1;
		}
	
		User_Motor_DataType->motor_l_2006.encoder_pos_all = User_Motor_DataType->motor_l_2006.encoder_pos_count*(MOTOR_USER_MAX_ENCODER+1) + User_Motor_DataType->motor_l_2006.encoder_pos_ecd;		
		#if 1
			temp_l = User_Motor_DataType->motor_l_2006.encoder_pos_all - User_Motor_DataType->motor_l_2006.encoder_pos_all_set;
			if( temp_l >= MOTOR_2006_ENCODER_HALF_ROUND )	temp_l = temp_l - MOTOR_2006_ENCODER_A_ROUND;
		#else
			temp = User_Motor_DataType->motor_l_2006.encoder_pos_all_set - User_Motor_DataType->motor_l_2006.encoder_pos_all;
			if( temp >= MOTOR_2006_ENCODER_HALF_ROUND )	temp = temp - MOTOR_2006_ENCODER_A_ROUND;	
		#endif

		if( temp_l > MOTOR_2006_ENCODER_ROUND_ERROR )
		{
			if( User_Motor_DataType->user_motor_RC->mouse.press_l )
			{
				User_Motor_DataType->motor_mode_l = MOTOR_GET_READY_MODE;
			}
			else if( User_Motor_DataType->motor_mode_l == MOTOR_SHOOT_MODE )
			{
				User_Motor_DataType->motor_mode_l = MOTOR_DONE_MODE;
			}
			else if( User_Motor_DataType->motor_mode_l == CLEAR_BULLET_STA_MODE )
			{
				User_Motor_DataType->motor_mode_l = CLEAR_BULLET_PRE_MODE;
			}
		}
	}


	if( User_Motor_DataType->motor_mode_r == MOTOR_SHOOT_MODE || User_Motor_DataType->motor_mode_r == CLEAR_BULLET_STA_MODE )
	{
		int temp_r;
		
		// ����ת�˼�Ȧ	
		if( User_Motor_DataType->motor_r_2006.encoder_pos_ecd - User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last >= MOTOR_USER_HALF_ENCODER )
		{
			User_Motor_DataType->motor_r_2006.encoder_pos_count--;
			//osDelay(5);
		}
		else if( User_Motor_DataType->motor_r_2006.encoder_pos_ecd - User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last <= -MOTOR_USER_HALF_ENCODER )
		{
			User_Motor_DataType->motor_r_2006.encoder_pos_count++;
			//osDelay(5);
		}
		
		User_Motor_DataType->motor_r_2006.encoder_pos_ecd_last = User_Motor_DataType->motor_r_2006.encoder_pos_ecd;
		
		if( User_Motor_DataType->motor_r_2006.encoder_pos_count >= MOTOR_2006_ENCODER_COUNT_MAX )
		{
			User_Motor_DataType->motor_r_2006.encoder_pos_count = 0;
		}
		else if( User_Motor_DataType->motor_r_2006.encoder_pos_count < 0 )
		{
			User_Motor_DataType->motor_r_2006.encoder_pos_count = MOTOR_2006_ENCODER_COUNT_MAX - 1;
		}
		
		User_Motor_DataType->motor_r_2006.encoder_pos_all = User_Motor_DataType->motor_r_2006.encoder_pos_count*(MOTOR_USER_MAX_ENCODER+1) + User_Motor_DataType->motor_r_2006.encoder_pos_ecd;
		

		#if 1
			temp_r = User_Motor_DataType->motor_r_2006.encoder_pos_all - User_Motor_DataType->motor_r_2006.encoder_pos_all_set;
			if( temp_r >= MOTOR_2006_ENCODER_HALF_ROUND )	temp_r = temp_r - MOTOR_2006_ENCODER_A_ROUND;
		#else
			temp = User_Motor_DataType->motor_r_2006.encoder_pos_all_set - User_Motor_DataType->motor_r_2006.encoder_pos_all;
			if( temp >= MOTOR_2006_ENCODER_HALF_ROUND )	temp = temp - MOTOR_2006_ENCODER_A_ROUND;	
		#endif
		if( temp_r > MOTOR_2006_ENCODER_ROUND_ERROR )
		{
			if( User_Motor_DataType->user_motor_RC->mouse.press_l )
			{
				User_Motor_DataType->motor_mode_r = MOTOR_GET_READY_MODE;
			}
			else if( User_Motor_DataType->motor_mode_r == MOTOR_SHOOT_MODE )
			{
				User_Motor_DataType->motor_mode_r = MOTOR_DONE_MODE;
			}
			else if( User_Motor_DataType->motor_mode_r == CLEAR_BULLET_STA_MODE )
			{
				User_Motor_DataType->motor_mode_r = CLEAR_BULLET_PRE_MODE;
			}
		}
	}
	
	// ������2006����ٶȶ�ȡ
	User_Motor_DataType->motor_l_2006.speed = User_Motor_DataType->motor_l_2006.motor_2006_measure->speed_rpm;
	User_Motor_DataType->motor_r_2006.speed = User_Motor_DataType->motor_r_2006.motor_2006_measure->speed_rpm;
	
	// ������2006����ٶ�����
	if( User_Motor_DataType->motor_mode_l == MOTOR_USER_STOP_MODE )
	{
		User_Motor_DataType->motor_l_2006.speed_set = 0;
		User_Motor_DataType->motor_r_2006.speed_set = 0;
	}
	else if( (User_Motor_DataType->motor_mode_l == MOTOR_SHOOT_MODE) || (User_Motor_DataType->motor_mode_l == MOTOR_GET_READY_MODE) )
	{
		User_Motor_DataType->motor_l_2006.speed_set = MOTER_2006_8_BULLETS_PERSEC;
		User_Motor_DataType->motor_r_2006.speed_set = -MOTER_2006_8_BULLETS_PERSEC;
	}
	else if( User_Motor_DataType->motor_mode_l == MOTOR_LOCKED_ROTOR )
	{
		User_Motor_DataType->motor_l_2006.speed_set = -MOTER_2006_8_BULLETS_PERSEC;
		User_Motor_DataType->motor_r_2006.speed_set = MOTER_2006_8_BULLETS_PERSEC;	
	}
	#if 0 //�ڱ�����Ҫ����
	else if( User_Motor_DataType->motor_mode_l == MOTOR_DONE_MODE )
	{
		User_Motor_DataType->motor_l_2006.speed_set = 0;
		User_Motor_DataType->motor_r_2006.speed_set = 0;
	}
	#endif
}
	

void Extra_3508_Speed_PID( User_Motor_t *User_Motor_PIDType )
{
	// ����Ħ����3508���PID����
	//uint8_t i;
	
	PID_calc( &User_Motor_PIDType->motor_extra_3508_high_l.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_high_l.speed, User_Motor_PIDType->motor_extra_3508_high_l.speed_set);
	User_Motor_PIDType->motor_extra_3508_high_l.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_high_l.motor_speed_pid.out;
	PID_calc( &User_Motor_PIDType->motor_extra_3508_low_l.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_low_l.speed, User_Motor_PIDType->motor_extra_3508_low_l.speed_set);
	User_Motor_PIDType->motor_extra_3508_low_l.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_low_l.motor_speed_pid.out;
	
	PID_calc( &User_Motor_PIDType->motor_extra_3508_high_r.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_high_r.speed, User_Motor_PIDType->motor_extra_3508_high_r.speed_set);
	User_Motor_PIDType->motor_extra_3508_high_r.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_high_r.motor_speed_pid.out;
	PID_calc( &User_Motor_PIDType->motor_extra_3508_low_r.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_low_r.speed, User_Motor_PIDType->motor_extra_3508_low_r.speed_set);
	User_Motor_PIDType->motor_extra_3508_low_r.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_low_r.motor_speed_pid.out;
}

void Motor_2006_PID_Set( User_Motor_t *User_Motor_PIDType )
{
	// һ��������2006���PID����
	PID_calc( &User_Motor_PIDType->motor_l_2006.motor_speed_pid, User_Motor_PIDType->motor_l_2006.speed, User_Motor_PIDType->motor_l_2006.speed_set);
	User_Motor_PIDType->motor_l_2006.given_current = (int16_t)User_Motor_PIDType->motor_l_2006.motor_speed_pid.out;
	PID_calc( &User_Motor_PIDType->motor_r_2006.motor_speed_pid, User_Motor_PIDType->motor_r_2006.speed, User_Motor_PIDType->motor_r_2006.speed_set);
	User_Motor_PIDType->motor_r_2006.given_current = (int16_t)User_Motor_PIDType->motor_r_2006.motor_speed_pid.out;
}

void Anti_locked_roter( User_Motor_t *User_Motor_DataType )
{
}	

void Anti_Locked_L_Callback( void const * argument )
{
	Locked_Time_Tick_L++;
}

void Anti_Locked_R_Callback( void const * argument )
{
	Locked_Time_Tick_R++;
}
