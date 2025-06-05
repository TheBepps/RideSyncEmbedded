#include "pid.h"

#ifdef PID_ERRORS_VECTOR
void pid_init(PidController_t *pid_controller, float kp, float ki, float kd,
              float sample_time, float anti_windUp, float *prev_errors,
              uint8_t n_prev_errors) {
  pid_controller->kp = kp;
  pid_controller->ki = ki;
  pid_controller->kd = kd;
  pid_controller->integrator = 0.0;
  pid_controller->prev_errors = prev_errors;
  pid_controller->n_prev_errors = n_prev_errors;
  pid_controller->prev_error_index = pid_controller->n_prev_errors - 1;
  for (int i = 0; i < pid_controller->n_prev_errors; ++i) {
    pid_controller->prev_errors[i] = 0.0;
  }
  pid_controller->error = 0.0;
  pid_controller->set_point = 0.0;
  pid_controller->sample_time = sample_time;
  pid_controller->anti_windUp = anti_windUp;
}
#else
void pid_init(PidController_t *pid_controller, float kp, float ki, float kd,
              float sample_time, float anti_windUp) {
  pid_controller->kp = kp;
  pid_controller->ki = ki;
  pid_controller->kd = kd;
  pid_controller->integrator = 0.0;
  pid_controller->prev_error = 0.0;
  pid_controller->error = 0.0;
  pid_controller->set_point = 0.0;
  pid_controller->sample_time = sample_time;
  pid_controller->anti_windUp = anti_windUp;
}
#endif

// Update errors vector and error, advance integrator
void pid_update(PidController_t *pid_controller, float status) {
#ifdef PID_ERRORS_VECTOR
  pid_controller->prev_error_index =
      (pid_controller->prev_error_index + 1) % pid_controller->n_prev_errors;
  pid_controller->prev_errors[pid_controller->prev_error_index] =
      pid_controller->error;
#else
  pid_controller->prev_error = pid_controller->prev_error;
#endif
  pid_controller->error = pid_controller->set_point - status;
  pid_controller->integrator +=
      pid_controller->error * pid_controller->sample_time;
}

float pid_compute(PidController_t *pid_controller) {
#ifdef PID_ERRORS_VECTOR
  uint8_t index =
      (pid_controller->prev_error_index + 1) % pid_controller->n_prev_errors;
  // Calculate filtered derivative
  float derivative =
      (pid_controller->error - pid_controller->prev_errors[index]) /
      (pid_controller->sample_time * pid_controller->n_prev_errors);
#else
  float derivative = (pid_controller->error - pid_controller->prev_error) /
                     pid_controller->sample_time;
#endif
  // Clamp integrator. Anti-windup is expressed in the same units as the
  // output, so we have to convert two times.
  if (pid_controller->integrator * pid_controller->ki >
      pid_controller->anti_windUp) {
    pid_controller->integrator =
        pid_controller->anti_windUp / pid_controller->ki;
  } else if (pid_controller->integrator * pid_controller->ki <
             -pid_controller->anti_windUp) {
    pid_controller->integrator =
        -pid_controller->anti_windUp / pid_controller->ki;
  }
  // Compute P, I, D terms
  float integral = pid_controller->ki * pid_controller->integrator;
  float value = pid_controller->kp * pid_controller->error + integral +
                pid_controller->kd * derivative;
  return value;
}

void pid_reset(PidController_t *pid_controller) {
  pid_controller->integrator = 0.0;
#ifdef PID_ERRORS_VECTOR
  for (int i = 0; i < pid_controller->n_prev_errors; ++i) {
    pid_controller->prev_errors[i] = 0.0;
  }
#else
  pid_controller->prev_error = 0.0;
#endif
}
