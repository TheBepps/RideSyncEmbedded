#include "temperature_controller.h"

#include <math.h>

/**
 * @brief This function initializes the temperature controller
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @param htim the handle of the timer that manages the PWM
 * @param channel the channel that outputs the PWM
 * @param inverted_pwm set as true for inverted logic (1.0f to turn off and 0.0f to set max power)
 */
void tc_init(temperature_controller_t *tc, TIM_HandleTypeDef *htim,
             uint32_t channel, bool inverted_pwm) {
  tc->htim = htim;
  tc->channel = channel;
  tc->inverted_pwm = inverted_pwm;
  pid_init(&tc->pid_controller, 0.25f, 0.01f, 0.2f, 1.0f, 0.5f, tc->prev_errors,
           TC_PREV_ERRORS_SIZE);
}

/**
 * @brief Set temperature to the temperature controller
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @param temperature target temperature in degrees Celsius
 */
void tc_set_temperature(temperature_controller_t *tc, float temperature) {
  tc->set_point = temperature;
  tc->pid_controller.set_point = temperature;
}

/**
 * @brief 
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @param temperature Current temperature measurement
 */
void tc_update(temperature_controller_t *tc, float temperature) {
  tc->last_temperature = temperature;
  pid_update(&tc->pid_controller, temperature);
  tc->commaded_duty_cycle = pid_compute(&tc->pid_controller);
  if (tc->inverted_pwm) {
    tc->commaded_duty_cycle = 1.0f - tc->commaded_duty_cycle;
  }
  tc->commaded_duty_cycle = fmaxf(0.0f, fminf(1.0f, tc->commaded_duty_cycle));
  set_pwm_duty_cycle(tc->htim, tc->channel, tc->commaded_duty_cycle);
}

bool tc_is_stabilized(temperature_controller_t *tc) {
  // Calculate mean
  float sum = 0.0f;
  for (int i = 0; i < TC_PREV_ERRORS_SIZE; ++i) {
    sum += tc->prev_errors[i];
  }
  float mean = sum / TC_PREV_ERRORS_SIZE;

  // Calculate standard deviation
  tc->std_dev = 0.0f;
  for (int i = 0; i < TC_PREV_ERRORS_SIZE; ++i) {
    tc->std_dev += (tc->prev_errors[i] - mean) * (tc->prev_errors[i] - mean);
  }
  tc->std_dev = sqrt(tc->std_dev / TC_PREV_ERRORS_SIZE);
  return tc->std_dev < TC_STD_DEV_THRESHOLD;
}

/**
 * @brief Checks if the target temperatures is in the target range with an error of 0.5 degrees Celsius
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @return true 
 * @return false 
 */
bool tc_is_at_temperature(temperature_controller_t *tc) {
  return fabs(tc->last_temperature - tc->pid_controller.set_point) < TC_TEMP_THRESHOLD;
}