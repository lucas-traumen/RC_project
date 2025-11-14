#ifndef PWM_H
#define PWM_H


#include "stm32f1xx_hal.h"



void handle_pwm__set_duty(TIM_HandleTypeDef *htim,uint32_t channel, uint8_t duty);

#endif

