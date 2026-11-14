#include <stdio.h>
#include <stdlib.h>
#include "foc.h"

int main(int argc, char **argv)
{
  float uq = argc > 1 ? (float)atof(argv[1]) : 6.0f;
  float d[3];

  printf("angle_deg,a,b,c\n");
  for (int k = 0; k <= 360; k += 2) {
    foc_phase_duties(uq, 0.0f, k * 3.14159265f / 180.0f, d);
    printf("%d,%.4f,%.4f,%.4f\n", k, d[0], d[1], d[2]);
  }
  return 0;
}
