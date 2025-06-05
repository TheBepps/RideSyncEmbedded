#ifndef TEMPERATURE_CONTROLLER_H
#define TEMPERATURE_CONTROLLER_H

#define PID_ERRORS_VECTOR
#include <stdbool.h>

#include "pid.h"
#include "pwm.h"
#include "tegc_config.h"

#define TC_PREV_ERRORS_SIZE 50
// Used to determine if the temperature is stabilized
#define TC_STD_DEV_THRESHOLD 0.2f
// Used to determine if the temperature is at the set point
#define TC_TEMP_THRESHOLD 0.5f

typedef struct temperature_controller {
  PidController_t pid_controller;
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  float set_point;
  float last_temperature;
  float prev_errors[TC_PREV_ERRORS_SIZE];
  float commaded_duty_cycle;
  float std_dev;
  bool inverted_pwm;
} temperature_controller_t;

/**
 * @brief Initialize the temperature controller
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @param htim the handle to the timer structure
 * @param channel the channel of the timer
 * @param inverted_pwm if the PWM signal is inverted
 */
void tc_init(temperature_controller_t *tc, TIM_HandleTypeDef *htim,
             uint32_t channel, bool inverted_pwm);
void tc_set_temperature(temperature_controller_t *tc, float temperature);
void tc_update(temperature_controller_t *tc, float temperature);
/**
 * @brief Check if the temperature is stabilized. It calculates the standard deviation of the last errors
 * and returns true if it is below the threshold (TC_STD_DEV_THRESHOLD)
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @return true if the temperature is stabilized
 */
bool tc_is_stabilized(temperature_controller_t *tc);
/**
 * @brief Check if the temperature is at the set point. It returns true if the difference between the set point
 * and the last temperature is below the threshold (TC_TEMP_THRESHOLD)
 * 
 * @param tc the handle to the temperature_controller_t structure
 * @return true if the temperature is at the set point
 */
bool tc_is_at_temperature(temperature_controller_t *tc);

#endif  // TEMPERATURE_CONTROLLER_H