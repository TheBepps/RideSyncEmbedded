#ifndef PWM_H
#define PWM_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

void set_pwm_duty_cycle(TIM_HandleTypeDef *htim, uint32_t channel,
                        float duty_cycle);

#endif  // PWM_H