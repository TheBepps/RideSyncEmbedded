#include "pwm.h"

/**
 * @brief Set the PWM duty cycle. Function managed internally by the temperature controller.
 * 
 * @param htim Timer handle that controls the PWM
 * @param channel Timer channel output of the PWM
 * @param duty_cycle PWM duty cycle (from 0.0 to 1.0). 
 * Please pay attention that this is HW duty cycle, not the logical one 
 * (see tc_init function for more information)
 */
void set_pwm_duty_cycle(TIM_HandleTypeDef *htim, uint32_t channel,
                        float duty_cycle) {
  __HAL_TIM_SET_COMPARE(htim, channel, duty_cycle * htim->Instance->ARR);
}