#ifndef MODEL_H
#define MODEL_H

#include <math.h>
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

static const Motor GM3506 = { 5.57, 0.068, 0.068, 5.0e-5, 1.0e-4 };

static inline void voltage_vector(float uq, float angle_el, double *mag, double *phase)
{
  float d[3];
  foc_phase_duties(uq, 0.0f, angle_el, d);
  double ua = d[0] * FOC_VBUS, ub = d[1] * FOC_VBUS, uc = d[2] * FOC_VBUS;
  double alpha = (2.0 * ua - ub - uc) / 3.0;
  double beta = (ub - uc) / sqrt(3.0);
  *mag = hypot(alpha, beta);
  *phase = atan2(beta, alpha);
}

static inline void motor_step(const Motor *m, State *s, double mag, double phase, double load, double dt)
{
  double rotor = FOC_POLE_PAIRS * s->theta + PI_D / 2.0;
  double vq = mag * sin(phase - rotor);
  double iq = (vq - m->ke * s->omega) / m->r;
  double torque = m->kt * iq - m->b * s->omega - load;
  s->omega += torque / m->j * dt;
  s->theta += s->omega * dt;
}

#endif
