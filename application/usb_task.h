/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      no action.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef USB_TASK_H
#define USB_TASK_H
#include "struct_typedef.h"
//#include "main.h"

typedef __packed struct 
{
  
  uint8_t header;
  uint8_t robot_color:1;
  uint8_t task_mode:2;
  uint8_t reserved:5;
  fp32 pitch;
  fp32 yaw;
  //uint16_t crc_check; 
  /* data */
}SendPacket_t;

typedef __packed struct 
{
  
  uint8_t header;
  uint8_t findEnemy:1;
  uint8_t target_color:1;
  uint8_t task_mode:2;
  uint8_t suggest_fire:1;
  uint8_t reserved:3;
  fp32 cmd_pitch;
  fp32 cmd_yaw;
  fp32 cmd_vx,cmd_vy,cmd_wz;
  //uint16_t crc_check; 
  /* data */
}RecievePacket_t;


extern void usb_task(void const * argument);

#endif
