/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGBµÆÐ§¡£
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

#endif
