#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#define FOC_VBUS           12.0f
#define FOC_VOLTAGE_LIMIT  7.0f
#define FOC_POLE_PAIRS     7.0f
#define FOC_LOOP_DT        0.001f

#define FOC_CMD_DEADZONE   5
#define FOC_RAMP_STEP      0.05f
#define FOC_TARGET_RATE    (0.0002f / FOC_POLE_PAIRS)
#define FOC_TARGET_LIMIT   1.57f

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_err;
  float out_limit;
} Pid;

typedef struct {
  float zero;
  float target;
  float speed;
  Pid pid;
} Axis;

typedef struct {
  float uq;
  float angle_el;
} FocCommand;

float foc_wrap_pi(float a);
float foc_constrain(float x, float lo, float hi);
float pid_update(Pid *p, float err);
void axis_init(Axis *a, float kp, float ki, float kd);
void axis_set_zero(Axis *a, float encoder_angle);
FocCommand axis_update(Axis *a, int8_t cmd, float encoder_angle);
void foc_phase_duties(float uq, float ud, float angle_el, float duty[3]);

#endif
