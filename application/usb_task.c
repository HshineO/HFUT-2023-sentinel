/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb ‰≥ˆ¥ÌŒÛ–≈œ¢
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"
#include "gimbal_task.h"
#include "referee.h"
#include "CRC8_CRC16.h"

#define BYTE0(dwTemp) (*(char*)(&dwTemp))
#define BYTE1(dwTemp) (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp)+3))

extern gimbal_control_t gimbal_control;

static void usb_printf(const char *fmt, ...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;

SendPacket_t sendPacket;
uint8_t send_buffer[12]={0};

RecievePacket_t recievePacket;
extern uint8_t usb_rx_buf[128];//[12];

void send_data();
void receive_data();


void usb_task(void const *argument)
{
    MX_USB_DEVICE_Init();
    // error_list_usb_local = get_error_list_point();

    while (1)
    {
        send_data();
		receive_data();
        osDelay(5);
    }
}

void send_data()
{
    sendPacket.header = 0x5A;

    sendPacket.robot_color = 1;
    sendPacket.task_mode = 1;
    sendPacket.yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
    sendPacket.pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
    
//    send_buffer[0]=sendPacket.header;
//    send_buffer[1]=sendPacket.robot_color;
//    send_buffer[2]=sendPacket.task_mode;
//    memcpy(send_buffer[3],&sendPacket.yaw,4);
//    memcpy(send_buffer[7],&sendPacket.pitch,4);
//    append_CRC16_check_sum(send_buffer,11+2);
    //send_buffer =(uint8_t *)(sendPacket);
    memcpy(send_buffer,&sendPacket,10);
	append_CRC16_check_sum(send_buffer,12);

    CDC_Transmit_FS(send_buffer, 12);//sizeof(&send_buffer));
}

void receive_data()
{
    if(usb_rx_buf[0]==0xA5)
    {
        if(verify_CRC16_check_sum(usb_rx_buf,(sizeof(recievePacket)+2)))
        {
            memcpy(&recievePacket,usb_rx_buf,sizeof(recievePacket));
        }
    }
}

static void usb_printf(const char *fmt, ...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}

//         usb_printf(
// "******************************\r\n\
// voltage percentage:%d%% \r\n\
// DBUS:%s\r\n\
// chassis motor1:%s\r\n\
// chassis motor2:%s\r\n\
// chassis motor3:%s\r\n\
// chassis motor4:%s\r\n\
// yaw motor:%s\r\n\
// pitch motor:%s\r\n\
// trigger motor:%s\r\n\
// gyro sensor:%s\r\n\
// accel sensor:%s\r\n\
// mag sensor:%s\r\n\
// referee usart:%s\r\n\
// ******************************\r\n",
//             get_battery_percentage(),
//             status[error_list_usb_local[DBUS_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
//             status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//             status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//             status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//             status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
//             status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
//             status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
//             status[error_list_usb_local[REFEREE_TOE].error_exist]);
