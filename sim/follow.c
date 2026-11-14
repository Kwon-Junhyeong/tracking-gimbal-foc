#include <stdio.h>
#include <stdlib.h>
#include "model.h"

#define UPDATE_MS 125

static const int8_t COMMANDS[] = {
  0, 0, 30, 60, 90, 110, 110, 90, 60, 30, 0, 0,
  0, 0, -40, -80, -100, -80, -40, 0, 0, 0, 0, 0
};

int main(int argc, char **argv)
{
  float kp = argc > 1 ? (float)atof(argv[1]) : 12.0f;
  float kd = argc > 2 ? (float)atof(argv[2]) : 0.1f;

  State s = { 0.0, 0.0 };
  Axis axis;
  const int n = (int)(sizeof COMMANDS / sizeof COMMANDS[0]);
  const int substeps = 20;
  const double dt = FOC_LOOP_DT / substeps;

  axis_init(&axis, kp, 0.0f, kd);
  axis_set_zero(&axis, 0.0f);

  printf("t,cmd,target,theta,uq\n");
  for (int k = 0; k < n * UPDATE_MS; k++) {
    int8_t cmd = COMMANDS[k / UPDATE_MS];
    FocCommand fc = axis_update(&axis, cmd, (float)s.theta);
    double mag, phase;
    voltage_vector(fc.uq, fc.angle_el, &mag, &phase);
    for (int i = 0; i < substeps; i++) motor_step(&GM3506, &s, mag, phase, 0.0, dt);
    printf("%.3f,%d,%.5f,%.5f,%.3f\n", k * FOC_LOOP_DT, cmd,
           axis.target * 180.0 / PI_D, s.theta * 180.0 / PI_D, fc.uq);
  }
  return 0;
}
