#ifndef FB_UTIL_H
#define FB_UTIL_H

typedef struct {
  double LA;
  double LB;
  double LC;
  double MA;
  double MB;
  double Mload;
  double IA;
  double IB;
  double sim_time;
  double load_force[2];
  double torque[2];
  double state[28];
  int working_mode;
  int assembly_mode;
  int state_valid;
} fb_setup;

fb_setup *make_fb_setup(double LA, double LB, double LC, double MA, double IA, double MB, double IB, double MLoad);
int fb_setup_is_xy_reachable(fb_setup *fb, double x, double y);
int fb_setup_is_angle_reachable(fb_setup *fb, double th1, double th2);
int fb_setup_tell_assembly_mode_at(fb_setup *fb, const double *xy, const double *ang);
int fb_setup_tell_assembly_mode(fb_setup *fb);
int fb_setup_tell_working_mode_at(fb_setup *fb, const double *xy, const double *ang);
int fb_setup_tell_working_mode(fb_setup *fb);
unsigned int fb_setup_tell_mode_at(fb_setup *fb, const double *xy, const double *ang);
unsigned int fb_setup_tell_mode(fb_setup *fb);
void fb_setup_ik(double *ang, fb_setup *fb, double x, double y, int working_mode);
void fb_setup_fk(double *xy, fb_setup *fb, double th1, double th2, int assembly_mode);
void fb_setup_jacobian_at(double *J, fb_setup *fb, const double *xy, const double *ang);
void fb_setup_jacobian(double *J, fb_setup *fb);
void fb_setup_coriolis_accel_at(double *accel, fb_setup *fb, const double *xy, const double *ang, const double *v);
void fb_setup_coriolis_accel(double *accel, fb_setup *fb);
double fb_setup_sgl2_condition_at(fb_setup *fb, double th1, double th2);
double fb_setup_sgl2_condition(fb_setup *fb);
void fb_setup_sgl2_condition_grad_at(double *res, fb_setup *fb, double th1, double th2);
void fb_setup_sgl2_condition_grad(double *res, fb_setup *fb);
void fb_setup_sgl2_condition_penalty_at(double *res, fb_setup *fb, double th1, double th2);
void fb_setup_sgl2_condition_penalty(double *res, fb_setup *fb);
int fb_setup_init_xy(fb_setup *fb, double x, double y, int working_mode);
int fb_setup_init_ang(fb_setup *fb, double th1, double th2, int assembly_mode);
void fb_setup_step(fb_setup *fb, double dt);

#endif