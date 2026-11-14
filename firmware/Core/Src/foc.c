#include "foc.h"
#include <math.h>
#include <stdlib.h>

#define PI_F      3.14159265358979f
#define TWO_PI_F  6.28318530717959f
#define SQRT3_2_F 0.86602540378444f

float foc_constrain(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float foc_wrap_pi(float a)
{
  while (a > PI_F) a -= TWO_PI_F;
  while (a < -PI_F) a += TWO_PI_F;
  return a;
}

float pid_update(Pid *p, float err)
{
  p->integral = foc_constrain(p->integral + err * FOC_LOOP_DT, -p->out_limit, p->out_limit);
  float derivative = (err - p->prev_err) / FOC_LOOP_DT;
  p->prev_err = err;
  return foc_constrain(p->kp * err + p->ki * p->integral + p->kd * derivative, -p->out_limit, p->out_limit);
}

void axis_init(Axis *a, float kp, float ki, float kd)
{
  a->zero = 0.0f;
  a->target = 0.0f;
  a->speed = 0.0f;
  a->pid.kp = kp;
  a->pid.ki = ki;
  a->pid.kd = kd;
  a->pid.integral = 0.0f;
  a->pid.prev_err = 0.0f;
  a->pid.out_limit = FOC_VOLTAGE_LIMIT;
}

void axis_set_zero(Axis *a, float encoder_angle)
{
  a->zero = encoder_angle;
}

static void ramp(float *speed, float target)
{
  float diff = target - *speed;
  if (diff > FOC_RAMP_STEP) *speed += FOC_RAMP_STEP;
  else if (diff < -FOC_RAMP_STEP) *speed -= FOC_RAMP_STEP;
  else *speed = target;
}

FocCommand axis_update(Axis *a, int8_t cmd, float encoder_angle)
{
  FocCommand out;
  float c = (abs(cmd) < FOC_CMD_DEADZONE) ? 0.0f : (float)cmd;

  ramp(&a->speed, c);
  a->target = foc_constrain(a->target + a->speed * FOC_TARGET_RATE, -FOC_TARGET_LIMIT, FOC_TARGET_LIMIT);

  float angle = foc_wrap_pi(encoder_angle - a->zero);
  out.uq = pid_update(&a->pid, foc_wrap_pi(a->target - angle));
  out.angle_el = angle * FOC_POLE_PAIRS + PI_F / 2.0f;
  return out;
}

void foc_phase_duties(float uq, float ud, float angle_el, float duty[3])
{
  uq = foc_constrain(uq, -FOC_VOLTAGE_LIMIT, FOC_VOLTAGE_LIMIT);

  float ca = cosf(angle_el);
  float sa = sinf(angle_el);
  float u_alpha = ca * ud - sa * uq;
  float u_beta  = sa * ud + ca * uq;

  float ua = u_alpha;
  float ub = -0.5f * u_alpha + SQRT3_2_F * u_beta;
  float uc = -0.5f * u_alpha - SQRT3_2_F * u_beta;

  float center = FOC_VBUS / 2.0f - (fmaxf(ua, fmaxf(ub, uc)) + fminf(ua, fminf(ub, uc))) / 2.0f;
  duty[0] = foc_constrain((ua + center) / FOC_VBUS, 0.0f, 1.0f);
  duty[1] = foc_constrain((ub + center) / FOC_VBUS, 0.0f, 1.0f);
  duty[2] = foc_constrain((uc + center) / FOC_VBUS, 0.0f, 1.0f);
}
