#include "bsp_buzzer.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
extern TIM_HandleTypeDef htim4;
extern osTimerId Buzz_TimerHandle;

void buzzer_on(uint16_t psc, uint16_t pwm)
{
	osTimerStart(Buzz_TimerHandle,1000);
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


void Buzz_Callback(void)
{
	buzzer_off();
}