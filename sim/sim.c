#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "foc.h"

#define PI_D 3.14159265358979

typedef struct {
  double r;
  double kt;
  double ke;
  double j;
  double b;
} Motor;

typedef struct {
  double theta;
  double omega;
} State;

typedef struct {
  float angle_el;
  float speed;
} OpenLoop;

static void voltage_vector(float uq, float angle_el, double *mag, double *phase)
{
  float d[3];
  foc_phase_duties(uq, 0.0f, angle_el, d);
  double ua = d[0] * FOC_VBUS, ub = d[1] * FOC_VBUS, uc = d[2] * FOC_VBUS;
  double alpha = (2.0 * ua - ub - uc) / 3.0;
  double beta = (ub - uc) / sqrt(3.0);
  *mag = hypot(alpha, beta);
  *phase = atan2(beta, alpha);
}

static void motor_step(const Motor *m, State *s, double mag, double phase, double load, double dt)
{
  double rotor = FOC_POLE_PAIRS * s->theta + PI_D / 2.0;
  double vq = mag * sin(phase - rotor);
  double iq = (vq - m->ke * s->omega) / m->r;
  double torque = m->kt * iq - m->b * s->omega - load;
  s->omega += torque / m->j * dt;
  s->theta += s->omega * dt;
}

static float open_loop_step(OpenLoop *o, int8_t cmd, float *angle_el)
{
  float c = (abs(cmd) < FOC_CMD_DEADZONE) ? 0.0f : (float)cmd;
  float diff = c - o->speed;
  if (diff > FOC_RAMP_STEP) o->speed += FOC_RAMP_STEP;
  else if (diff < -FOC_RAMP_STEP) o->speed -= FOC_RAMP_STEP;
  else o->speed = c;
  o->angle_el += o->speed * 0.0002f;
  *angle_el = o->angle_el;
  if (fabsf(o->speed) > 0.01f) return foc_constrain(3.0f + fabsf(o->speed) * 0.1f, 3.0f, 5.0f);
  return 0.5f;
}

int main(int argc, char **argv)
{
  float kp = argc > 1 ? (float)atof(argv[1]) : 12.0f;
  float kd = argc > 2 ? (float)atof(argv[2]) : 0.1f;
  double load_peak = argc > 3 ? atof(argv[3]) : 0.03;

  Motor m = { 5.57, 0.068, 0.068, 5.0e-5, 1.0e-4 };
  State closed = { 0.0, 0.0 };
  State open = { 0.0, 0.0 };
  Axis axis;
  OpenLoop ol = { 0.0f, 0.0f };
  const int substeps = 20;
  const double dt = FOC_LOOP_DT / substeps;

  axis_init(&axis, kp, 0.0f, kd);
  axis_set_zero(&axis, 0.0f);

  printf("t,cmd,load,target_closed,theta_closed,uq_closed,target_open,theta_open,u_open\n");
  for (int k = 0; k < 2000; k++) {
    double t = k * FOC_LOOP_DT;
    int8_t cmd = (t >= 0.2 && t < 0.6) ? 60 : 0;
    double load = (t >= 1.2 && t < 1.3) ? load_peak : 0.0;

    FocCommand fc = axis_update(&axis, cmd, (float)closed.theta);
    double mag_c, ph_c;
    voltage_vector(fc.uq, fc.angle_el, &mag_c, &ph_c);

    float ol_angle;
    float ol_u = open_loop_step(&ol, cmd, &ol_angle);
    double mag_o, ph_o;
    voltage_vector(ol_u, ol_angle, &mag_o, &ph_o);

    for (int i = 0; i < substeps; i++) {
      motor_step(&m, &closed, mag_c, ph_c, load, dt);
      motor_step(&m, &open, mag_o, ph_o, load, dt);
    }

    printf("%.3f,%d,%.4f,%.5f,%.5f,%.3f,%.5f,%.5f,%.2f\n", t, cmd, load,
           axis.target * 180.0 / PI_D, closed.theta * 180.0 / PI_D, fc.uq,
           ol.angle_el / FOC_POLE_PAIRS * 180.0 / PI_D, open.theta * 180.0 / PI_D, ol_u);
  }
  return 0;
}
