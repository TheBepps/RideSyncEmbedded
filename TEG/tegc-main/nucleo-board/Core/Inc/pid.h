#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

/**
 * @brief The PID_ERRORS_VECTOR stores in a circular buffer the n_prev_errors obtained
 * while calculating the PID. It is useful to filter the error signal that is used to
 * calculate the derivative term. (usually is very noisy)
 */
#define PID_ERRORS_VECTOR

typedef struct pidController_t {
  float kp;
  float ki;
  float kd;
  float integrator;
  float error;
  float sample_time;
  float set_point;
  float anti_windUp;
#ifdef PID_ERRORS_VECTOR
  uint8_t n_prev_errors;
  int prev_error_index;
  float *prev_errors;
#else
  float prev_error;
#endif
} PidController_t;


/**
 * @brief Initialize the PID controller
 * 
 * @param pid_controller the handle to the PID controller
 * @param kp the proportional gain
 * @param ki the integral gain
 * @param kd the derivative gain
 * @param sample_time the time between each update
 * @param anti_windUp the maximum value of the integrator such that it does not
 * take the whole output range
 * @param prev_errors the circular buffer to store the previous errors
 * @param n_prev_errors the size of the circular buffer
 */

#ifdef PID_ERRORS_VECTOR
void pid_init(PidController_t *pid_controller, float kp, float ki, float kd,
              float sample_time, float anti_windUp, float *prev_errors,
              uint8_t n_prev_errors);
#else
void pid_init(PidController_t *pid_controller, float kp, float ki, float kd,
              float sample_time, float anti_windUp);
#endif

/**
 * @brief Update the PID controller with the current status
 * 
 * @param pid_controller the handle to the PID controller
 * @param status the current status of the system (temperature, position, etc)
 */
void pid_update(PidController_t *pid_controller, float status);
/**
 * @brief Compute the PID controller output
 */
float pid_compute(PidController_t *pid_controller);
void pid_reset(PidController_t *pid_controller);

#endif
