#include <stdio.h>
#include "fb_util.h"

#define LA 1.0
#define LB 0.8
#define LC 0.7

int main(void)
{
  fb_setup *fb = make_fb_setup(LA, LB, LC, 0.1, 0.1, 1.0, 1.0, 0.5);
  fb_setup_init_xy(fb, 0.3, 1.2, 2);
  double v[2] = {-1.0, -0.5};
  double xy[2] = {fb->state[6], fb->state[7]};
  double dt = 0.001;
  double ang_last[2] = {fb->state[2], fb->state[13]};
  
  for (int i=0; i<20; i++) {
    double ang[2] = {fb->state[2], fb->state[13]};
    double accel_computed[2];
    fb_setup_coriolis_accel_at(accel_computed, fb, xy, ang, v);
    printf("accel: %.3f %.3f\tang_diff: %.7f %.7f\n", accel_computed[0], accel_computed[1], (ang[0]-ang_last[0])/dt, (ang[1]-ang_last[1])/dt);
    ang_last[0] = ang[0]; ang_last[1] = ang[1];
    xy[0] += dt*v[0];
    xy[1] += dt*v[1];
    fb_setup_init_xy(fb, xy[0], xy[1], 2);
  }
  // double accel_diff[2];
  
  return 0;
}