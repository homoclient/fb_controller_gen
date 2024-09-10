#include "fb_util.h"
#include "simple_linalg.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define PI 3.1415926535897932

fb_setup *make_fb_setup(double LA, double LB, double LC, double MA, double IA, double MB, double IB, double MLoad)
{
  if (LA<0.0 || LB<0.0 || LC <0.0 || MA<0.0 || MB<0.0 || IA<0.0 || IB<0.0 || MLoad<0.0) return NULL;
  fb_setup *fb = malloc(sizeof(fb_setup));
  fb->LA=LA; fb->LB=LB; fb->LC=LC; fb->MA=MA; fb->MB=MB; fb->IA=IA; fb->IB=IB; fb->Mload=MLoad;
  for (int i=0; i<28; i++) fb->state[i] = 0.0;
  fb->state_valid = 0;
  return fb;
}

int fb_setup_is_xy_reachable(fb_setup *fb, double x, double y)
{
  double d1s = (x+fb->LC/2.0)*(x+fb->LC/2.0) + y*y;
  double d2s = (x-fb->LC/2.0)*(x-fb->LC/2.0) + y*y;
  double far = (fb->LA+fb->LB)*(fb->LA+fb->LB);
  double near = (fb->LA-fb->LB)*(fb->LA-fb->LB);
  return d1s<far && d1s>near && d2s<far && d2s>near;
}

int fb_setup_is_angle_reachable(fb_setup *fb, double th1, double th2)
{
  double s1 = sin((th1+th2)/2.0);
  double s2 = sin((th1-th2)/2.0);
  return 4*fb->LA*fb->LA - fb->LC*fb->LC - 4*fb->LB*fb->LC*s1*s2 - 4*fb->LB*fb->LB*s2*s2 > 0.0;
}

int fb_setup_tell_assembly_mode_at(fb_setup *fb, const double *xy, const double *ang)
{
  double r1px = xy[0] - (-fb->LC/2.0 + cos(ang[0])*fb->LB);
  double r1py = xy[1] - sin(ang[0])*fb->LB;
  double r2px = xy[0] - (fb->LC/2.0 + cos(ang[1])*fb->LB);
  double r2py = xy[1] - sin(ang[1])*fb->LB;
  return r1px*r2py - r1py*r2px > 0.0 ? 1 : -1;
}

int fb_setup_tell_assembly_mode(fb_setup *fb)
{
  assert(fb->state_valid);
  double *xy = fb->state + 6;
  double ang[2] = {fb->state[2], fb->state[13]};
  return fb_setup_tell_assembly_mode_at(fb, xy, ang);
}

int fb_setup_tell_working_mode_at(fb_setup *fb, const double *xy, const double *ang)
{
  double r1px = xy[0] - (-fb->LC/2.0 + cos(ang[0])*fb->LB);
  double r1py = xy[1] - sin(ang[0])*fb->LB;
  double r2px = xy[0] - (fb->LC/2.0 + cos(ang[1])*fb->LB);
  double r2py = xy[1] - sin(ang[1])*fb->LB;
  double r1x = cos(ang[0])*fb->LB;
  double r1y = sin(ang[0])*fb->LB;
  double r2x = cos(ang[1])*fb->LB;
  double r2y = sin(ang[1])*fb->LB;
  int wm1 = r1x*r1py - r1y*r1px > 0;
  int wm2 = r2x*r2py - r2y*r2px > 0;
  return wm1 | wm2<<1;
}

int fb_setup_tell_working_mode(fb_setup *fb)
{
  assert(fb->state_valid);
  double *xy = fb->state + 6;
  double ang[2] = {fb->state[2], fb->state[13]};
  return fb_setup_tell_working_mode_at(fb, xy, ang);
}

unsigned int fb_setup_tell_mode_at(fb_setup *fb, const double *xy, const double *ang)
{
  double r1px = xy[0] - (-fb->LC/2.0 + cos(ang[0])*fb->LB);
  double r1py = xy[1] - sin(ang[0])*fb->LB;
  double r2px = xy[0] - (fb->LC/2.0 + cos(ang[1])*fb->LB);
  double r2py = xy[1] - sin(ang[1])*fb->LB;
  double r1x = cos(ang[0])*fb->LB;
  double r1y = sin(ang[0])*fb->LB;
  double r2x = cos(ang[1])*fb->LB;
  double r2y = sin(ang[1])*fb->LB;
  unsigned int wm1 = r1x*r1py - r1y*r1px > 0;
  unsigned int wm2 = r2x*r2py - r2y*r2px > 0;
  unsigned int am = r1px*r2py - r1py*r2px > 0.0;
  return wm1 | wm2<<1 | am<<2;
}

unsigned int fb_setup_tell_mode(fb_setup *fb)
{
  assert(fb->state_valid);
  double *xy = fb->state + 6;
  double ang[2] = {fb->state[2], fb->state[13]};
  return fb_setup_tell_mode_at(fb, xy, ang);
}

void fb_setup_ik(double *ang, fb_setup *fb, double x, double y, int working_mode)
{
  assert(working_mode >= 0 || working_mode <= 3);
  assert(fb_setup_is_xy_reachable(fb, x, y));
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  const double d1s = (x+LC/2.0)*(x+LC/2.0) + y*y;
  const double d2s = (x-LC/2.0)*(x-LC/2.0) + y*y;
  const double tmp1 = LB*LB-LA*LA+d1s;
  const double tmp1c = (working_mode&1 ? 1.0 : -1.0)*sqrt(4.0*LB*LB*d1s - tmp1*tmp1);
  const double tmp2 = LB*LB-LA*LA+d2s;
  const double tmp2c = (working_mode&2 ? 1.0 : -1.0)*sqrt(4.0*LB*LB*d2s - tmp2*tmp2);
  ang[0] = atan2(tmp1*y+tmp1c*(-x-LC/2.0), tmp1*(x+LC/2.0)+tmp1c*y);
  ang[1] = atan2(tmp2*y+tmp2c*(-x+LC/2.0), tmp2*(x-LC/2.0)+tmp2c*y);
}

void fb_setup_fk(double *xy, fb_setup *fb, double th1, double th2, int assembly_mode)
{
  assert(assembly_mode >= -1 || assembly_mode <= 1);
  assert(fb_setup_is_angle_reachable(fb, th1, th2));
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  assembly_mode = assembly_mode > 0 ? 1 : -1;
  double D = (1.0-cos(th1-th2))*LB*LB/2.0+(cos(th2)-cos(th1))*LB*LC/2.0 + LC*LC/4.0;
  double tmp = LB*(cos((th1-th2)/2.0) + assembly_mode*sqrt(LA*LA/D-1.0)*sin((th1-th2)/2.0));
  xy[0] = tmp*cos((th1+th2)/2.0);
  xy[1] = tmp*sin((th1+th2)/2.0)+assembly_mode*sqrt(LA*LA/D-1.0)*LC/2.0;
}

// void fb_setup_jacobian_at(double *J, fb_setup *fb, const double *xy, const double *ang)
// {
//   // const double LA = fb->LA;
//   const double LB = fb->LB;
//   const double LC = fb->LC;
//   const double sgl_tol = 1e-3;
//   double diag[2] = {sin(ang[0])*(xy[0]+LC/2.0) - xy[1]*cos(ang[0]), -sin(ang[1])*(xy[0]-LC/2.0) + xy[1]*cos(ang[1])};
//   double den = LB*sin(ang[1]-ang[0])+xy[0]*(sin(ang[0])-sin(ang[1]))+xy[1]*(cos(ang[1])-cos(ang[0])+LC/LB)-(sin(ang[0])+sin(ang[1]))*LC/2.0;
//   // assert(fabs(den)>sgl_tol);
//   //assert(fabs(diag[0])>sgl_tol && fabs(diag[1])>sgl_tol);
//   if (fabs(den) > sgl_tol) {
//     int am = fb_setup_tell_assembly_mode_at(fb, xy, ang);
//     J[0] = am*(LB*sin(ang[1])-xy[1])*diag[0]/den;
//     J[1] = am*(LB*sin(ang[0])-xy[1])*diag[1]/den;
//     J[2] = am*(-LB*cos(ang[1])+xy[0]-LC/2.0)*diag[0]/den;
//     J[3] = am*(-LB*cos(ang[0])+xy[0]+LC/2.0)*diag[1]/den;
//   } else {
//     J[0] = 0.0;
//     J[1] = 0.0;
//     J[2] = 0.0;
//     J[3] = 0.0;
//   }
// }

void fb_setup_jacobian_at(double *J, fb_setup *fb, const double *xy, const double *ang)
{
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  const double r1px = xy[0] - LB*cos(ang[0]) + LC/2.0;
  const double r1py = xy[1] - LB*sin(ang[0]);
  const double r2px = xy[0] - LB*cos(ang[1]) - LC/2.0;
  const double r2py = xy[1] - LB*sin(ang[1]);
  const double den = r1px*r2py - r1py*r2px;
  assert(fabs(den) > 1e-3);
  const double v1 = LB*(sin(ang[0])*r1px-cos(ang[0])*r1py)/den;
  const double v2 = -LB*(sin(ang[1])*r2px-cos(ang[1])*r2py)/den;
  J[0] = -v1 * r2py;
  J[2] = v1 * r2px;
  J[1] = -v2 * r1py;
  J[3] = v2 * r1px;
}

void fb_setup_jacobian(double *J, fb_setup *fb)
{
  assert(fb->state_valid);
  double *xy = fb->state + 6;
  double ang[2] = {fb->state[2], fb->state[13]};
  fb_setup_jacobian_at(J, fb, xy, ang);
}

// void fb_setup_coriolis_accel_at(double *accel, fb_setup *fb, const double *xy, const double *ang, const double *v)
// {
//   unsigned int mode = fb_setup_tell_mode_at(fb, xy, ang);
//   const double LA = fb->LA;
//   const double LB = fb->LB;
//   const double LC = fb->LC;
//   const double LAs = LA*LA;
//   const double LBs = LB*LB;
//   const double LAss = LAs*LAs;
//   const double LBss = LBs*LBs;
//   const double LAsss = LAs*LAss;
//   const double d1s = (xy[0]+LC/2.0)*(xy[0]+LC/2.0) + xy[1]*xy[1];
//   const double d2s = (xy[0]-LC/2.0)*(xy[0]-LC/2.0) + xy[1]*xy[1];
//   const double d1 = sqrt(d1s);
//   const double d2 = sqrt(d2s);
//   const double dd1s = (v[0]*(xy[0]+LC/2.0) + v[1]*xy[1])*(v[0]*(xy[0]+LC/2.0) + v[1]*xy[1])/d1s;
//   const double dd2s = (v[0]*(xy[0]-LC/2.0) + v[1]*xy[1])*(v[0]*(xy[0]-LC/2.0) + v[1]*xy[1])/d2s;
//   const double ddd1 = (v[0]*v[0]-v[1]*v[1]-dd1s)/d1;
//   const double ddd2 = (v[0]*v[0]-v[1]*v[1]-dd2s)/d2;
//   const double ddphi[2] = {
//     2.0*(v[0]*xy[1]-v[1]*(xy[0]+LC/2.0))*(v[0]*(xy[0]+LC/2.0)+v[1]*xy[1])/(d1s*d1s),
//     2.0*(v[0]*xy[1]-v[1]*(xy[0]-LC/2.0))*(v[0]*(xy[0]-LC/2.0)+v[1]*xy[1])/(d2s*d2s)
//   };
//   const double dpsi1dd1 = (LAs-LBs+d1s)/(2.0*LB*d1s*sqrt(1.0-(-LAs+LBs+d1s)*(-LAs+LBs+d1s)/(4.0*LBs*d1s)));
//   const double dpsi2dd2 = (LAs-LBs+d2s)/(2.0*LB*d2s*sqrt(1.0-(-LAs+LBs+d2s)*(-LAs+LBs+d2s)/(4.0*LBs*d2s)));
//   const double LBs_d1s_3 = (LBs-d1s)*(LBs-d1s)*(LBs-d1s);
//   const double LBs_d2s_3 = (LBs-d2s)*(LBs-d2s)*(LBs-d2s);
//   double tmp1 = 2.0*LAs*(LBs+d1s)-LAss-(LBs-d1s)*(LBs-d1s);
//   tmp1 = sqrt(tmp1*tmp1*tmp1);
//   double tmp2 = 2.0*LAs*(LBs+d2s)-LAss-(LBs-d2s)*(LBs-d2s);
//   tmp2 = sqrt(tmp2*tmp2*tmp2);
//   double ddpsi1ddd1 = (LAsss-LBs_d1s_3-LAss*(3.0*LBs+5.0*d1s)+LAs*(3.0*LBss+2.0*LBs*d1s+3.0*d1s*d1s))/(d1s*tmp1);
//   double ddpsi2ddd2 = (LAsss-LBs_d2s_3-LAss*(3.0*LBs+5.0*d2s)+LAs*(3.0*LBss+2.0*LBs*d2s+3.0*d2s*d2s))/(d2s*tmp2);
//   const double ddpsi[2] = {
//     (mode&1 ? 1.0 : -1.0)*(ddpsi1ddd1*dd1s+dpsi1dd1*ddd1),
//     (mode&2 ? 1.0 : -1.0)*(ddpsi2ddd2*dd2s+dpsi2dd2*ddd2)
//   };
//   accel[0] = ddphi[0] + ddpsi[0];
//   accel[1] = ddphi[1] + ddpsi[1];
// }

static double _ddpsi(double x, double y, double vx, double vy, double A, double B)
{
  const double As = A*A;
  const double Bs = B*B;
  const double vs = vx*vx + vy*vy;
  const double ds = x*x + y*y;
  const double vdotps = (vx*x + vy*y)*(vx*x + vy*y);
  const double tmp1 = -As + Bs + ds;
  const double tmp2 = Bs*ds*(1.0 - tmp1*tmp1/(4*Bs*ds));
  const double num = -vs + vs*tmp1/(2.0*ds) + 2.0*vdotps/ds - 1.5*vdotps*tmp1/(ds*ds) - (-2.0+tmp1/ds)*(-2.0+tmp1/ds)*vdotps*tmp1/(8*tmp2);
  return -num/sqrt(tmp2);
}

void fb_setup_coriolis_accel_at(double *accel, fb_setup *fb, const double *xy, const double *ang, const double *v)
{
  unsigned int mode = fb_setup_tell_mode_at(fb, xy, ang);
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  const double d1s = (xy[0]+LC/2.0)*(xy[0]+LC/2.0) + xy[1]*xy[1];
  const double d2s = (xy[0]-LC/2.0)*(xy[0]-LC/2.0) + xy[1]*xy[1];
  const double ddphi[2] = {
    2.0*(v[0]*xy[1]-v[1]*(xy[0]+LC/2.0))*(v[0]*(xy[0]+LC/2.0)+v[1]*xy[1])/(d1s*d1s),
    2.0*(v[0]*xy[1]-v[1]*(xy[0]-LC/2.0))*(v[0]*(xy[0]-LC/2.0)+v[1]*xy[1])/(d2s*d2s)
  };
  const double ddpsi[2] = {
    (mode&1 ? 1.0 : -1.0)*_ddpsi(xy[0]+LC/2.0, xy[1], v[0], v[1], LA, LB),
    (mode&2 ? 1.0 : -1.0)*_ddpsi(xy[0]-LC/2.0, xy[1], v[0], v[1], LA, LB)
  };
  accel[0] = ddphi[0] + ddpsi[0];
  accel[1] = ddphi[1] + ddpsi[1];
}

void fb_setup_coriolis_accel(double *accel, fb_setup *fb)
{
  assert(fb->state_valid);
  double *xy = fb->state + 6;
  double ang[2] = {fb->state[2], fb->state[13]};
  double *v = fb->state + (6+14);
  fb_setup_coriolis_accel_at(accel, fb, xy, ang, v);
}

double fb_setup_sgl2_condition_at(fb_setup *fb, double th1, double th2)
{
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  double dr1r2s = LB*LB*((cos(th1)-cos(th2))*(cos(th1)-cos(th2)) + (sin(th1)-sin(th2))*(sin(th1)-sin(th2))) - 2*LB*LC*(cos(th1)-cos(th2)) + LC*LC;
  return sin(PI*dr1r2s/(4*LA*LA));
}

double fb_setup_sgl2_condition(fb_setup *fb)
{
  assert(fb->state_valid);
  return fb_setup_sgl2_condition_at(fb, fb->state[2], fb->state[13]);
}

void fb_setup_sgl2_condition_grad_at(double *res, fb_setup *fb, double th1, double th2)
{
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  double dr1r2s = LB*LB*((cos(th1)-cos(th2))*(cos(th1)-cos(th2)) + (sin(th1)-sin(th2))*(sin(th1)-sin(th2))) - 2*LB*LC*(cos(th1)-cos(th2)) + LC*LC;
  double tmp = PI*LB*cos(PI*dr1r2s/(4*LA*LA))/(2.0*LA*LA);
  res[0] = tmp*(LB*sin(th1-th2) + LC*sin(th1));
  res[1] = -tmp*(LB*sin(th1-th2) + LC*sin(th2));
}

void fb_setup_sgl2_condition_grad(double *res, fb_setup *fb)
{
  assert(fb->state_valid);
  return fb_setup_sgl2_condition_grad_at(res, fb, fb->state[2], fb->state[13]);
}

void fb_setup_sgl2_condition_penalty_at(double *res, fb_setup *fb, double th1, double th2)
{
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
  double dr1r2s = LB*LB*((cos(th1)-cos(th2))*(cos(th1)-cos(th2)) + (sin(th1)-sin(th2))*(sin(th1)-sin(th2))) - 2*LB*LC*(cos(th1)-cos(th2)) + LC*LC;
  double tmp = PI*LB/(2.0*LA*LA*tan(PI*dr1r2s/(4*LA*LA)));
  res[0] = tmp*(LB*sin(th1-th2) + LC*sin(th1));
  res[1] = -tmp*(LB*sin(th1-th2) + LC*sin(th2));
}

void fb_setup_sgl2_condition_penalty(double *res, fb_setup *fb)
{
  assert(fb->state_valid);
  return fb_setup_sgl2_condition_penalty_at(res, fb, fb->state[2], fb->state[13]);
}

int fb_setup_init_xy(fb_setup *fb, double x, double y, int working_mode)
{
  if (fb == NULL) return 0;
  if (!fb_setup_is_xy_reachable(fb, x, y)) return 0;
  double ang[2];
  fb_setup_ik(ang, fb, x, y, working_mode);
  double r1x = cos(ang[0])*fb->LB;
  double r1y = sin(ang[0])*fb->LB;
  double r2x = cos(ang[1])*fb->LB;
  double r2y = sin(ang[1])*fb->LB;
  fb->state[0] = -fb->LC/2.0 + r1x/2.0;
  fb->state[1] = r1y/2.0;
  fb->state[2] = ang[0];
  fb->state[3] = (x + r1x - fb->LC/2.0)/2.0;
  fb->state[4] = (y + r1y)/2.0;
  fb->state[5] = atan2(y-r1y, x-(r1x-fb->LC/2.0));
  fb->state[6] = x;
  fb->state[7] = y;
  fb->state[8] = (x + r2x + fb->LC/2.0)/2.0;
  fb->state[9] = (y + r2y)/2.0;
  fb->state[10] = atan2(y-r2y, x-(r2x+fb->LC/2.0));
  fb->state[11] = fb->LC/2.0 + r2x/2.0;
  fb->state[12] = r2y/2.0;
  fb->state[13] = ang[1];
  for (int i=14; i<28; i++) fb->state[14] = 0.0;
  fb->load_force[0] = 0.0;
  fb->load_force[1] = 0.0;
  fb->torque[0] = 0.0;
  fb->torque[1] = 0.0;
  fb->sim_time = 0.0;
  fb->state_valid = 1;
  fb->working_mode = fb_setup_tell_working_mode(fb);
  fb->assembly_mode = fb_setup_tell_assembly_mode(fb);
  return 1;
}

int fb_setup_init_ang(fb_setup *fb, double th1, double th2, int assembly_mode)
{
  if (fb == NULL) return 0;
  if (!fb_setup_is_angle_reachable(fb, th1, th2)) return 0;
  double xy[2];
  fb_setup_fk(xy, fb, th1, th2, assembly_mode);
  double r1x = cos(th1)*fb->LB;
  double r1y = sin(th1)*fb->LB;
  double r2x = cos(th2)*fb->LB;
  double r2y = sin(th2)*fb->LB;
  fb->state[0] = -fb->LC/2.0 + r1x/2.0;
  fb->state[1] = r1y/2.0;
  fb->state[2] = th1;
  fb->state[3] = (xy[0] + r1x - fb->LC/2.0)/2.0;
  fb->state[4] = (xy[1] + r1y)/2.0;
  fb->state[5] = atan2(xy[1]-r1y, xy[0]-(r1x-fb->LC/2.0));
  fb->state[6] = xy[0];
  fb->state[7] = xy[1];
  fb->state[8] = (xy[0] + r2x + fb->LC/2.0)/2.0;
  fb->state[9] = (xy[1] + r2y)/2.0;
  fb->state[10] = atan2(xy[1]-r2y, xy[0]-(r2x+fb->LC/2.0));
  fb->state[11] = fb->LC/2.0 + r2x/2.0;
  fb->state[12] = r2y/2.0;
  fb->state[13] = th2;
  for (int i=14; i<28; i++) fb->state[14] = 0.0;
  fb->load_force[0] = 0.0;
  fb->load_force[1] = 0.0;
  fb->torque[0] = 0.0;
  fb->torque[1] = 0.0;
  fb->sim_time = 0.0;
  fb->state_valid = 1;
  fb->working_mode = fb_setup_tell_working_mode(fb);
  fb->assembly_mode = fb_setup_tell_assembly_mode(fb);
  return 1;
}

static void _fb_constraint_force(double *force, fb_setup *fb, double *w)
{
  double *x = w != NULL ? w : fb->state;
  double *dx = w != NULL ? w+14 : fb->state+14;
  double LA = fb->LA;
  double LB = fb->LB;
  // double LC = fb->LC;
  double J[168] = {
//  |<----- Bar LB ----->| |<------ Bar LA ------>|  |Load| |<------ Bar RA ------->| |<------ Bar RB ------->|
    1, 0,  sin(x[2])*LB/2,  0,  0,               0,  0,  0,  0,  0,                0,  0,  0,                0,
    0, 1, -cos(x[2])*LB/2,  0,  0,               0,  0,  0,  0,  0,                0,  0,  0,                0,
    1, 0, -sin(x[2])*LB/2, -1,  0, -sin(x[5])*LA/2,  0,  0,  0,  0,                0,  0,  0,                0,
    0, 1,  cos(x[2])*LB/2,  0, -1,  cos(x[5])*LA/2,  0,  0,  0,  0,                0,  0,  0,                0,
    0, 0,               0,  1,  0, -sin(x[5])*LA/2, -1,  0,  0,  0,                0,  0,  0,                0,
    0, 0,               0,  0,  1,  cos(x[5])*LA/2,  0, -1,  0,  0,                0,  0,  0,                0,
    0, 0,               0,  0,  0,               0,  1,  0, -1,  0,  sin(x[10])*LA/2,  0,  0,                0,
    0, 0,               0,  0,  0,               0,  0,  1,  0, -1, -cos(x[10])*LA/2,  0,  0,                0,
    0, 0,               0,  0,  0,               0,  0,  0,  1,  0,  sin(x[10])*LA/2, -1,  0,  sin(x[13])*LB/2,
    0, 0,               0,  0,  0,               0,  0,  0,  0,  1, -cos(x[10])*LA/2,  0, -1, -cos(x[13])*LB/2,
    0, 0,               0,  0,  0,               0,  0,  0,  0,  0,                0,  1,  0,  sin(x[13])*LB/2,
    0, 0,               0,  0,  0,               0,  0,  0,  0,  0,                0,  0,  1, -cos(x[13])*LB/2
  };

  double dJ[168] = {
//  |<-------- Bar LB -------->| |<-------- Bar LA -------->| load  |<-------- Bar RA --------->| |<-------- Bar RB --------->|
    0, 0,  cos(x[2])*dx[2]*LB/2, 0, 0,                     0, 0, 0, 0, 0,                      0, 0, 0,                      0,
    0, 0,  sin(x[2])*dx[2]*LB/2, 0, 0,                     0, 0, 0, 0, 0,                      0, 0, 0,                      0,
    0, 0, -cos(x[2])*dx[2]*LB/2, 0, 0, -cos(x[5])*dx[5]*LA/2, 0, 0, 0, 0,                      0, 0, 0,                      0,
    0, 0, -sin(x[2])*dx[2]*LB/2, 0, 0, -sin(x[5])*dx[5]*LA/2, 0, 0, 0, 0,                      0, 0, 0,                      0,
    0, 0,                     0, 0, 0, -cos(x[5])*dx[5]*LA/2, 0, 0, 0, 0,                      0, 0, 0,                      0,
    0, 0,                     0, 0, 0, -sin(x[5])*dx[5]*LA/2, 0, 0, 0, 0,                      0, 0, 0,                      0,
    0, 0,                     0, 0, 0,                     0, 0, 0, 0, 0, cos(x[10])*dx[10]*LA/2, 0, 0,                      0,
    0, 0,                     0, 0, 0,                     0, 0, 0, 0, 0, sin(x[10])*dx[10]*LA/2, 0, 0,                      0,
    0, 0,                     0, 0, 0,                     0, 0, 0, 0, 0, cos(x[10])*dx[10]*LA/2, 0, 0, cos(x[13])*dx[13]*LB/2,
    0, 0,                     0, 0, 0,                     0, 0, 0, 0, 0, sin(x[10])*dx[10]*LA/2, 0, 0, sin(x[13])*dx[13]*LB/2,
    0, 0,                     0, 0, 0,                     0, 0, 0, 0, 0,                      0, 0, 0, cos(x[13])*dx[13]*LB/2,
    0, 0,                     0, 0, 0,                     0, 0, 0, 0, 0,                      0, 0, 0, sin(x[13])*dx[13]*LB/2
  };

  double J_T[168];
  transposed(J_T, J, 12, 14);

  const double W[14] = {1.0/fb->MB, 1.0/fb->MB, 1.0/fb->IB, 1.0/fb->MA, 1.0/fb->MA, 1.0/fb->IA, 1.0/fb->Mload, 1.0/fb->Mload, 1.0/fb->MA, 1.0/fb->MA, 1.0/fb->IA, 1.0/fb->MB, 1.0/fb->MB, 1.0/fb->IB};
  double W_J_T[168];
  matrowscaled(W_J_T, J_T, W, 14, 12);
  double J_W_J_T[144];
  matmuld(J_W_J_T, J, W_J_T, 12, 14, 14, 12);
  double J_W_J_T_inv[144];
  double tmp[12];
  unsigned int index_tmp[12];
  invd_gaussian(J_W_J_T_inv, J_W_J_T, 12, tmp, index_tmp);

  double dJ_dx[12];
  matvecmuld(dJ_dx, dJ, dx, 12, 14);
  double external_accel[14] = {0, 0, fb->torque[0]/fb->IB, 0, 0, 0, fb->load_force[0]/fb->Mload, fb->load_force[1]/fb->Mload, 0, 0, 0, 0, 0, fb->torque[1]/fb->IB};
  double J_F[12];
  matvecmuld(J_F, J, external_accel, 12, 14);
  double B[12];
  vecsumd(B, dJ_dx, J_F, 12);
  vecnegated(B, 12);
  double sol[12];
  matvecmuld(sol, J_W_J_T_inv, B, 12, 12);
  matvecmuld(force, J_T, sol, 14, 12);
}

static void _fb_setup_dw(double *dw, fb_setup *fb, double *w)
{
  w = w != NULL ? w : fb->state;
  memcpy((void *)dw, (void *)(w+14), 14*sizeof(double));
  double force[14];
  _fb_constraint_force(force, fb, w);
  force[2] += fb->torque[0];
  force[6] += fb->load_force[0];
  force[7] += fb->load_force[1];
  force[13] += fb->torque[1];
  double W[14] = {1.0/fb->MB, 1.0/fb->MB, 1.0/fb->IB, 1.0/fb->MA, 1.0/fb->MA, 1.0/fb->IA, 1.0/fb->Mload, 1.0/fb->Mload, 1.0/fb->MA, 1.0/fb->MA, 1.0/fb->IA, 1.0/fb->MB, 1.0/fb->MB, 1.0/fb->IB};
  for (int i=0; i<14; i++) {
    dw[14+i] = force[i]*W[i];
  }
}

void fb_setup_step(fb_setup *fb, double dt)
{
  assert(fb != NULL || fb->state_valid);
  double k1[28], k2[28], k3[28], k4[28], w_tmp[28];
  _fb_setup_dw(k1, fb, NULL);
  for (int i=0; i<28; i++) w_tmp[i] = fb->state[i] + (dt*0.5)*k1[i];
  _fb_setup_dw(k2, fb, w_tmp);
  for (int i=0; i<28; i++) w_tmp[i] = fb->state[i] + (dt*0.5)*k2[i];
  _fb_setup_dw(k3, fb, w_tmp);
  for (int i=0; i<28; i++) w_tmp[i] = fb->state[i] + dt*k3[i];
  _fb_setup_dw(k4, fb, w_tmp);
  for (int i=0; i<28; i++) fb->state[i] += dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])*(1.0/6.0);
  fb->state[0] = -fb->LC/2.0 + cos(fb->state[2])*fb->LB/2.0;
  fb->state[1] = sin(fb->state[2])*fb->LB/2.0;
  fb->state[3] = (-fb->LC/2.0 + cos(fb->state[2])*fb->LB + fb->state[6])/2.0;
  fb->state[4] = (sin(fb->state[2])*fb->LB + fb->state[7])/2.0;
  fb->state[8] = (fb->LC/2.0 + cos(fb->state[13])*fb->LB + fb->state[6])/2.0;
  fb->state[9] = (sin(fb->state[13])*fb->LB + fb->state[7])/2.0;
  fb->state[11] = fb->LC/2.0 + cos(fb->state[13])*fb->LB/2.0;
  fb->state[12] = sin(fb->state[13])*fb->LB/2.0;
  fb->working_mode = fb_setup_tell_working_mode(fb);
  fb->assembly_mode = fb_setup_tell_assembly_mode(fb);
  fb->sim_time += dt;
  fb->load_force[0] = 0.0;
  fb->load_force[1] = 0.0;
  fb->torque[0] = 0.0;
  fb->torque[1] = 0.0;
}