#include <stdio.h>
#include <math.h>
// #include <unistd.h>
#include "usleep.h"
#include <pthread.h>
#include <assert.h>
#include "fb_util.h"
#include "raylib.h"

// gcc -lm -lopengl32 -pthread fb_util.c simple_linalg.c fb_sim_3.c usleep.c raylib.dll

#define WINDOW_W 1024
#define WINDOW_H 768

typedef struct {
  int mode_switching;
  unsigned int from;
  unsigned int to;
  int switching_arm;
  double mode_switching_position[2];
  double from_angle[2];
  double to_angle[2];
} controller_memory;

typedef struct {
  fb_setup *fb;
  void (*controller)(double *torque,
    const double *angle, const double *angular_velocity, const double *load_xy, const double *load_velocity,
    const double *target_xy, controller_memory *memory,
    void (*virtual_force)(double *force, const double *load_xy, const double *load_velocity, const double *target_xy),
    void (*compensation_force)(double *force, const double *load_xy, const double *load_velocity));
  controller_memory *memory;
  double target_xy[2];
  void (*virtual_force)(double *force, const double *load_xy, const double *load_velocity, const double *target_xy);
  void (*compensation_force)(double *force, const double *load_xy, const double *load_velocity);
  double dt;
  int running;
} fb_sim;

void *fb_sim_runner(void *p)
{
  fb_setup *fb = ((fb_sim *)p)->fb;
  void (*fb_controller)(double *torque,
    const double *angle, const double *angular_velocity, const double *load_xy, const double *load_velocity,
    const double *target_xy, controller_memory *memory,
    void (*virtual_force)(double *force, const double *load_xy, const double *load_velocity, const double *target_xy),
    void (*compensation_force)(double *force, const double *load_xy, const double *load_velocity)) = ((fb_sim *)p)->controller;
  void (*virtual_force)(double *force, const double *load_xy, const double *load_velocity, const double *target_xy) =
    ((fb_sim *)p)->virtual_force;
  void (*compensation_force)(double *force, const double *load_xy, const double *load_velocity) =
    ((fb_sim *)p)->compensation_force;
  while (1) {
    if (((fb_sim *)p)->running) {
      const double ang[2] = {fb->state[2], fb->state[13]};
      const double w[2] = {fb->state[2+14], fb->state[13+14]};
      const double *load_xy = fb->state + 6;
      const double *load_v = fb->state + 6 + 14;
      const double *target = ((fb_sim *)p)->target_xy;
      fb_controller(fb->torque, ang, w, load_xy, load_v, target, ((fb_sim *)p)->memory, virtual_force, compensation_force);
      fb_setup_step(fb, ((fb_sim *)p)->dt);  
    }
    useconds_t usecs = (unsigned int)(((fb_sim *)p)->dt*0.5e6);
    usleep(usecs);
  }
  pthread_exit(NULL);
}

double wrap_angle(double x)
{
  while (x > 3.1415926535897932) x -= 2*3.1415926535897932;
  while (x < -3.1415926535897932) x += 2*3.1415926535897932;
  return x;
}

#include "workspace_sdf_lut.txt"

double sdf_lut_sample(double x, double y, unsigned int mode)
{
  assert(fabs(x)<SDF_LUT_W*SDF_LUT_GRID_LENGTH/2.0 && fabs(y)<SDF_LUT_H*SDF_LUT_GRID_LENGTH/2.0);
  unsigned int grid_x, grid_y;
  switch (mode) {
    case 0:
      grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
    case 1:
      grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[1][grid_x+grid_y*SDF_LUT_W];
    case 2:
      grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[2][grid_x+grid_y*SDF_LUT_W];
    case 3:
      grid_x = (unsigned int)(-x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
    case 4:
      grid_x = (unsigned int)(-x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
    case 5:
      grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[2][grid_x+grid_y*SDF_LUT_W];
    case 6:
      grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[1][grid_x+grid_y*SDF_LUT_W];
    case 7:
      grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN);
      grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN);
      return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
  }
  return 0.0; // not reached
}

double sdf_pick_direction(double *end_pos, const double *start_pos, const double *target_pos, double threshold, unsigned int mode)
{
  const double dir_angle[14] = {0.393, -0.393, 0.785, -0.785, 1.178, -1.178, 1.571, -1.571, 1.963, -1.963, 2.356, -2.356, 2.749, -2.749};
  const double dir_weight[14] = {0.962, 0.962, 0.854, 0.854, 0.691, 0.691, 0.5, 0.5, 0.309, 0.309, 0.146, 0.146, 0.038, 0.038};
  double cur[2] = {start_pos[0], start_pos[1]};
  double start_dst = sdf_lut_sample(cur[0], cur[1], mode) - threshold;
  double cur_dst = start_dst;
  if (cur_dst < 0.0) {
    end_pos[0] = start_pos[0];
    end_pos[1] = start_pos[1];
    return 0.0;
  }
  double dst_los_x = target_pos[0]-start_pos[0];
  double dst_los_y = target_pos[1]-start_pos[1];
  double dst_los = sqrt(dst_los_x*dst_los_x + dst_los_y*dst_los_y);
  double ang_center = atan2(target_pos[1]-start_pos[1], target_pos[0]-start_pos[0]);
  double dst_traveled = 0.0;
  double dir_center[2] = {cos(ang_center), sin(ang_center)};
  const unsigned int max_iter = 16;
  for (int i=0; i<max_iter; i++) {
    dst_traveled += cur_dst;
    if (dst_traveled > dst_los) {
      end_pos[0] = target_pos[0];
      end_pos[1] = target_pos[1];
      return dst_los;
    }
    cur[0] += dir_center[0]*cur_dst;
    cur[1] += dir_center[1]*cur_dst;
    cur_dst = sdf_lut_sample(cur[0], cur[1], mode) - threshold;
    if (cur_dst < 0.0) break;
  }
  double dst_max = dst_traveled;
  double dst_weighted_max = dst_traveled;
  double end_chosen[2] = {cur[0], cur[1]};
  if (mode != 1 && mode != 6) goto MODE_023457;
  double blind_direction = start_pos[0]*dir_center[1]-start_pos[1]*dir_center[0] > 0.0 ? 1.0 : -1.0;
  for (int i=0; i<14; i++) {
    double angle = ang_center + dir_angle[i];
    double dir[2] = {cos(angle), sin(angle)};
    double cur[2] = {start_pos[0], start_pos[1]};
    double blind_dst = (cur[0]*dir_center[1]-cur[1]*dir_center[0])*blind_direction +0.2;
    double cur_dst = fmin(sdf_lut_sample(cur[0], cur[1], mode), blind_dst) - threshold;
    double dst_traveled = 0.0;
    for (int j=0; j<max_iter; j++) {
      dst_traveled += cur_dst;
      cur[0] += dir[0]*cur_dst;
      cur[1] += dir[1]*cur_dst;
      blind_dst = (cur[0]*dir_center[1]-cur[1]*dir_center[0])*blind_direction +0.2;
      cur_dst = fmin(sdf_lut_sample(cur[0], cur[1], mode), blind_dst) - threshold;
      if (cur_dst < 0.0) break;
    }
    double dst_weighted = dst_traveled*dir_weight[i];
    if (dst_weighted > dst_weighted_max) {
      dst_weighted_max = dst_weighted;
      dst_max = dst_traveled;
      end_chosen[0] = cur[0];
      end_chosen[1] = cur[1];
    }
  }
  end_pos[0] = end_chosen[0];
  end_pos[1] = end_chosen[1];
  return dst_max;

MODE_023457:
  for (int i=0; i<14; i++) {
    double angle = ang_center + dir_angle[i];
    double dir[2] = {cos(angle), sin(angle)};
    double cur[2] = {start_pos[0], start_pos[1]};
    double cur_dst = start_dst;
    double dst_traveled = 0.0;
    for (int j=0; j<max_iter; j++) {
      dst_traveled += cur_dst;
      cur[0] += dir[0]*cur_dst;
      cur[1] += dir[1]*cur_dst;
      cur_dst = sdf_lut_sample(cur[0], cur[1], mode) - threshold;
      if (cur_dst < 0.0) break;
    }
    double dst_weighted = dst_traveled*dir_weight[i];
    if (dst_weighted > dst_weighted_max) {
      dst_weighted_max = dst_weighted;
      dst_max = dst_traveled;
      end_chosen[0] = cur[0];
      end_chosen[1] = cur[1];
    }
  }

  end_pos[0] = end_chosen[0];
  end_pos[1] = end_chosen[1];
  return dst_max;
}

static void _mode_handling(double *res, const double *xy, const double *ang, double LA, double LB, double LC, unsigned int to_mode, double frac)
{
  // const double wm_grad1 = (to_mode&1 ? -1.0 : 1.0)*((xy[0]-LB*cos(ang[0])+LC/2.0)*cos(ang[0]) + (xy[1]-LB*sin(ang[0]))*sin(ang[0]))/LA;
  // const double wm_grad2 = (to_mode&2 ? -1.0 : 1.0)*((xy[0]-LB*cos(ang[1])-LC/2.0)*cos(ang[1]) + (xy[1]-LB*sin(ang[1]))*sin(ang[1]))/LA;
  // const double wm_mag = sqrt(wm_grad1*wm_grad1 + wm_grad2*wm_grad2);
  double wm_grad1 = (to_mode&1 ? -1.0 : 1.0)*((xy[0]-LB*cos(ang[0])+LC/2.0)*cos(ang[0]) + (xy[1]-LB*sin(ang[0]))*sin(ang[0]));
  double wm_grad2 = (to_mode&2 ? -1.0 : 1.0)*((xy[0]-LB*cos(ang[1])-LC/2.0)*cos(ang[1]) + (xy[1]-LB*sin(ang[1]))*sin(ang[1]));
  if (fabs(wm_grad1) > fabs(wm_grad2)) {
    wm_grad1 = copysign(1.0, wm_grad1);
    wm_grad2 = 0.0;
  } else {
    wm_grad1 = 0.0;
    wm_grad2 = copysign(1.0, wm_grad2);
  }
  const double dr1r2s = LB*LB*((cos(ang[0])-cos(ang[1]))*(cos(ang[0])-cos(ang[1]))+(sin(ang[0])-sin(ang[1]))*(sin(ang[0])-sin(ang[1]))) - 2*LB*LC*(cos(ang[0])-cos(ang[1])) + LC*LC;
  const double am_grad1 = 2*LB*(LB*sin(ang[0]-ang[1])+LC*sin(ang[0]));
  const double am_grad2 = -2*LB*(LB*sin(ang[0]-ang[1])+LC*sin(ang[1]));
  const double am_mag = sqrt(am_grad1*am_grad1 + am_grad2*am_grad2);
  res[0] = frac*wm_grad1 + (1.0-frac)*am_grad1/am_mag;
  res[1] = frac*wm_grad2 + (1.0-frac)*am_grad2/am_mag;
}

const unsigned int working_mode_border_type[8] = {3, 1, 3, 1, 1, 3, 1, 3};

const double working_mode_border_center[16] = {
  -2.47744, -0.12856,
  -2.47744, 0.0,
  -0.66415, -3.01303,
  -0.66415, 0.0,
  0.66415, 0.0,
  0.66415, 3.01303,
  2.47744, 0.0,
  2.47744, 0.12756
};

const double working_mode_border_range[16] = {
  1.78213, 1.24654,
  1.35946, 0.0,
  1.78213, 1.24654,
  1.35946, 0.0,
  1.35946, 0.0,
  1.78213, 1.24654,
  1.35946, 0.0,
  1.78213, 1.24654,
};

double working_mode_border_position(double *res, const double *from, const double *to, unsigned int from_mode, unsigned int to_mode)
{
  const double LA = 1.0;
  const double LB = 0.8;
  const double LC = 0.7;
  const double safe_distance = 0.04;
  const double safety_factor_inner = 0.5;
  const double safety_factor_outer = 0.9;
  const double inner_radius = fabs(LA-LB) + safe_distance;
  const double outer_radius = LA+LB - safe_distance;
  if (from_mode == to_mode) {
    const double r_left = sqrt((from[0]+LC/2.0)*(from[0]+LC/2.0) + from[1]*from[1]);
    const double r_right = sqrt((from[0]-LC/2.0)*(from[0]-LC/2.0) + from[1]*from[1]);
    const double dst[4] = {fabs(r_left-inner_radius), fabs(r_left-outer_radius), fabs(r_right-inner_radius), fabs(r_right-outer_radius)};
    int min = 0;
    double dst_min = dst[0];
    for (int i=0; i<4; i++) {
      if (dst[i] < dst_min) {
        min = i;
        dst_min = dst[i];
      }
    }
    const double radius = min & 1 ? outer_radius : inner_radius;
    const double arg = atan2(from[1], from[0] + ((min & 2) ? -LC/2.0 : LC/2.0));
    res[0] = radius*cos(arg) + ((min & 2) ? LC/2.0 : -LC/2.0);
    res[1] = radius*sin(arg);
    return dst_min;
  }
  const unsigned int border_id = (from_mode + to_mode)/2 + (((from_mode + to_mode)&7) == 5);
  assert(working_mode_border_type[border_id] > 0 && working_mode_border_type[border_id] <= 3);
  const double x_shift = (border_id+1)&2 ? -LC/2.0 : LC/2.0;
  const double from_x = from[0] + x_shift;
  const double from_y = from[1];
  const double mag_from = sqrt(from_x*from_x + from_y*from_y);
  const double to_x = to[0] + x_shift;
  const double to_y = to[1];
  const double mag_to = sqrt(to_x*to_x + to_y*to_y);
  double border_position_inner[2];
  double border_position_outer[2];
  if (working_mode_border_type[border_id] & 1) {
    const double ref_arg = atan2(from_y*fabs(mag_to-inner_radius)+to_y*fabs(mag_from-inner_radius), from_x*fabs(mag_to-inner_radius)+to_x*fabs(mag_from-inner_radius));
    const double ref_to_inner_center = wrap_angle(ref_arg - working_mode_border_center[2*border_id+0]);
    if (fabs(ref_to_inner_center) < safety_factor_inner*working_mode_border_range[2*border_id+0]) {
      border_position_inner[0] = inner_radius*cos(ref_arg) - x_shift;
      border_position_inner[1] = inner_radius*sin(ref_arg);
    } else {
      const double arg = copysign(safety_factor_inner*working_mode_border_range[2*border_id+0],ref_to_inner_center) + working_mode_border_center[2*border_id+0];
      border_position_inner[0] = inner_radius*cos(arg) - x_shift;
      border_position_inner[1] = inner_radius*sin(arg);
    }
  }
  if (working_mode_border_type[border_id] & 2) {
    const double ref_arg = atan2(from_y*fabs(mag_to-outer_radius)+to_y*fabs(mag_from-outer_radius), from_x*fabs(mag_to-outer_radius)+to_x*fabs(mag_from-outer_radius));
    const double ref_to_outer_center = wrap_angle(ref_arg - working_mode_border_center[2*border_id+1]);
    if (fabs(ref_to_outer_center) < safety_factor_outer*working_mode_border_range[2*border_id+1]) {
      border_position_outer[0] = outer_radius*cos(ref_arg) - x_shift;
      border_position_outer[1] = outer_radius*sin(ref_arg);
    } else {
      const double arg = copysign(safety_factor_outer*working_mode_border_range[2*border_id+1],ref_to_outer_center) + working_mode_border_center[2*border_id+1];
      border_position_outer[0] = outer_radius*cos(arg) - x_shift;
      border_position_outer[1] = outer_radius*sin(arg);
    }
  }
  double distance;
  switch (working_mode_border_type[border_id]) {
    case 1: // only has inner border
      distance = sqrt((from[0]-border_position_inner[0])*(from[0]-border_position_inner[0]) + (from[1]-border_position_inner[1])*(from[1]-border_position_inner[1]))
        + sqrt((to[0]-border_position_inner[0])*(to[0]-border_position_inner[0]) + (to[1]-border_position_inner[1])*(to[1]-border_position_inner[1]));
      res[0] = border_position_inner[0];
      res[1] = border_position_inner[1];
    break;
    case 2: // only has outer border
      distance = sqrt((from[0]-border_position_outer[0])*(from[0]-border_position_outer[0]) + (from[1]-border_position_outer[1])*(from[1]-border_position_outer[1]))
        + sqrt((to[0]-border_position_outer[0])*(to[0]-border_position_outer[0]) + (to[1]-border_position_outer[1])*(to[1]-border_position_outer[1]));
      res[0] = border_position_outer[0];
      res[1] = border_position_outer[1];
    break;
    case 3: // has both
      double distance_inner = sqrt((from[0]-border_position_inner[0])*(from[0]-border_position_inner[0]) + (from[1]-border_position_inner[1])*(from[1]-border_position_inner[1]))
        + sqrt((to[0]-border_position_inner[0])*(to[0]-border_position_inner[0]) + (to[1]-border_position_inner[1])*(to[1]-border_position_inner[1]));
      double distance_outer = sqrt((from[0]-border_position_outer[0])*(from[0]-border_position_outer[0]) + (from[1]-border_position_outer[1])*(from[1]-border_position_outer[1]))
        + sqrt((to[0]-border_position_outer[0])*(to[0]-border_position_outer[0]) + (to[1]-border_position_outer[1])*(to[1]-border_position_outer[1]));
      if (distance_inner < distance_outer) {
        distance = distance_inner;
        res[0] = border_position_inner[0];
        res[1] = border_position_inner[1];
      } else {
        distance = distance_outer;
        res[0] = border_position_outer[0];
        res[1] = border_position_outer[1];
      }
    break;
    default: // not reached
      assert(0);
  }
  return distance;
}

void wm_crossing_direction(double *res, const double *J, unsigned int from, unsigned int to, unsigned int side, unsigned int inner_outer)
{
  if (inner_outer == 0) { // inner
    const double k = 0.99;
    if (side == 0) { // switch left
      double v[2] = {J[1], J[3]};
      double v_mag = sqrt(v[0]*v[0] + v[1]*v[1]);
      v[0] /= v_mag; v[1] /= v_mag;
      switch (to) {
        case 1:
        case 2:
        case 5:
        case 6:
          // res[0] = J[1] - k*J[0]; res[1] = J[3] - k*J[2]; return;
          res[0] = (1.0-k)*v[0] - k*J[0]; res[1] = (1.0-k)*v[1] - k*J[2]; return;
        case 0:
        case 3:
        case 4:
        case 7:
          // res[0] = -J[1] + k*J[0]; res[1] = -J[3] + k*J[2]; return;
          res[0] = -(1.0-k)*v[0] + k*J[0]; res[1] = -(1.0-k)*v[1] + k*J[2]; return;
        default:
          assert(0);
      }
    } else { // switch right
      double v[2] = {J[0], J[2]};
      double v_mag = sqrt(v[0]*v[0] + v[1]*v[1]);
      v[0] /= v_mag; v[1] /= v_mag;
      switch (to) {
        case 1:
        case 2:
        case 5:
        case 6:
          // res[0] = -J[0]; res[1] = -J[2]; return;
          res[0] = -(1.0-k)*v[0] + k*J[1]; res[1] = -(1.0-k)*v[1] + k*J[3]; return;
        case 0:
        case 3:
        case 4:
        case 7:
          // res[0] = J[0]; res[1] = J[2]; return;
          res[0] = (1.0-k)*v[0] - k*J[1]; res[1] = (1.0-k)*v[1] - k*J[3]; return;
        default:
          assert(0);
      }
    }
  } else { // outer
    if (side == 0) { // switch left
      switch (to) {
        case 0:
        case 1:
        case 4:
        case 5:
          if (from != to) {
            res[0] = -J[3]; res[1] = J[1]; return;
          } else {
            res[0] = J[3]; res[1] = -J[1]; return;
          }
        case 2:
        case 3:
        case 6:
        case 7:
          if (from != to) {
            res[0] = J[3]; res[1] = -J[1]; return;
          } else {
            res[0] = -J[3]; res[1] = J[1]; return;
          }
        default:
          assert(0);
      }
    } else { // switch right
      switch (to) {
        case 0:
        case 1:
        case 4:
        case 5:
          if (from != to) {
            res[0] = J[2]; res[1] = -J[0]; return;
          } else {
            res[0] = -J[2]; res[1] = J[0]; return;
          }
        case 2:
        case 3:
        case 6:
        case 7:
          if (from != to) {
            res[0] = -J[2]; res[1] = J[0]; return;
          } else {
            res[0] = J[2]; res[1] = -J[0]; return;
          }
        default:
          assert(0);
      }
    }
  }
}

void controller_0(double *torque,
  const double *angle, const double *angular_velocity, const double *load_xy, const double *load_velocity,
  const double *target_xy, controller_memory *memory,
  void (*virtual_force)(double *force, const double *load_xy, const double *load_velocity, const double *target_xy),
  void (*compensation_force)(double *force, const double *load_xy, const double *load_velocity))
{
  const double LA = 1.0; const double LB = 0.8; const double LC = 0.7;
  const double LAs = LA*LA; const double LBs = LB*LB;
  // const double MB = 0.1;
  // const double IB = 0.1;
  const double MB = 1.0; const double IB = 1.0; const double MLoad = 1.0;
  const double IM = IB + MB*LB*LB/4.0;
  const double user_safe_distance = 0.1;
  const double sdf_threshold = 0.02;
  const double mode_switching_position_threshold = 0.02;
  const double forced_mode_switching_threshold = 0.1;
  const double mode_switching_angular_speed_threshold = 1.0;
  const double mode_switching_kp = 10.0;
  const double assembly_mode_ambiguous_threshold = 5e-3;
  const double r1x = LB*cos(angle[0]);
  const double r1y = LB*sin(angle[0]);
  const double r2x = LB*cos(angle[1]);
  const double r2y = LB*sin(angle[1]);
  const double r1px = load_xy[0] - r1x + LC/2.0;
  const double r1py = load_xy[1] - r1y;
  const double r2px = load_xy[0] - r2x - LC/2.0;
  const double r2py = load_xy[1] - r2y;
  const double r1pcorssr2p = r1px*r2py - r1py*r2px;
  const double r1crossr1p = r1x*r1py - r1y*r1px;
  const double r2crossr2p = r2x*r2py - r2y*r2px;
  const double vcrossr1 = load_velocity[0]*r1y - load_velocity[1]*r1x;
  const double vcrossr2 = load_velocity[0]*r2y - load_velocity[1]*r2x;
  const double r1dotr1p = r1x*r1px + r1y*r1py;
  const double r2dotr2p = r2x*r2px + r2y*r2py;
  const double vdotr1p = load_velocity[0]*r1px + load_velocity[1]*r1py;
  const double vdotr2p = load_velocity[0]*r2px + load_velocity[1]*r2py;
  const double magvs = load_velocity[0]*load_velocity[0] + load_velocity[1]*load_velocity[1];

  const unsigned int current_mode = (r1crossr1p > 0) | (r2crossr2p > 0)<<1 | (r1pcorssr2p > 0)<<2;
  unsigned int target_mode;
  double mode_switching_position[2];
  // const double sgl2_condition = fabs(r1pcorssr2p/(LA*LA));
  const double J_v1 = -r1crossr1p/r1pcorssr2p;
  const double J_v2 = r2crossr2p/r1pcorssr2p;
  const double J[4] = {-J_v1*r2py, -J_v2*r1py, J_v1*r2px, J_v2*r1px};
  
  double fc[2];
  if (compensation_force != NULL) {
    compensation_force(fc, load_xy, load_velocity);
  } else {
    fc[0] = 0.0; fc[1] = 0.0;
  }
  double torque_fc[2] = {J[0]*fc[0] + J[2]*fc[1], J[1]*fc[0] + J[3]*fc[1]};
  double fv[2];
  double torque_fv[2];
  double alpha[2];
  double torque_mode_switching[2];

  do {
    if (sdf_lut_sample(target_xy[0], target_xy[1], current_mode) > user_safe_distance) {
      target_mode = current_mode;
      break;
    }
    int target_in_adjacent_modes = 0;
    double mode_switching_distance_1;
    double temp_target_1[2];
    const unsigned int adjacent_mode_1 = current_mode ^ 1;
    if (sdf_lut_sample(target_xy[0], target_xy[1], adjacent_mode_1) > user_safe_distance) {
      target_in_adjacent_modes |= 1;
      mode_switching_distance_1 = working_mode_border_position(temp_target_1, load_xy, target_xy, current_mode, adjacent_mode_1);
    }
    double mode_switching_distance_2;
    double temp_target_2[2];
    const unsigned int adjacent_mode_2 = current_mode ^ 2;
    if (sdf_lut_sample(target_xy[0], target_xy[1], adjacent_mode_2) > user_safe_distance) {
      target_in_adjacent_modes |= 2;
      mode_switching_distance_2 = working_mode_border_position(temp_target_2, load_xy, target_xy, current_mode, adjacent_mode_2);
    }
    switch (target_in_adjacent_modes) {
      case 0:
        printf("halt for having no way to reach target\ncurrent mode: %u\n", current_mode);
        goto halt;
      break;
      case 1:
        mode_switching_position[0] = temp_target_1[0]; mode_switching_position[1] = temp_target_1[1];
        target_mode = adjacent_mode_1;
      break;
      case 2:
        mode_switching_position[0] = temp_target_2[0]; mode_switching_position[1] = temp_target_2[1];
        target_mode = adjacent_mode_2;
      break;
      case 3:
        if (mode_switching_distance_1 < mode_switching_distance_2) {
        mode_switching_position[0] = temp_target_1[0]; mode_switching_position[1] = temp_target_1[1];
        target_mode = adjacent_mode_1;
      } else {
        mode_switching_position[0] = temp_target_2[0]; mode_switching_position[1] = temp_target_2[1];
        target_mode = adjacent_mode_2;
      }
      break;
      default: // not reached
    }
  } while (0);
  if (memory->mode_switching) goto mode_switching;
  int forced_mode_switching;
  int switching_arm;
  if (fabs(r1crossr1p) < forced_mode_switching_threshold) {
    forced_mode_switching = 1;
    switching_arm = 0;
  } else if (fabs(r2crossr2p) < forced_mode_switching_threshold) {
    forced_mode_switching = 1;
    switching_arm = 1;
  } else {
    forced_mode_switching = 0;
  }
  if (forced_mode_switching){
    working_mode_border_position(mode_switching_position, load_xy, load_xy, current_mode, current_mode);
    memory->from = current_mode;
    memory->to = current_mode;
    const double d1s = (mode_switching_position[0]+LC/2.0)*(mode_switching_position[0]+LC/2.0) + mode_switching_position[1]*mode_switching_position[1];
    const double d2s = (mode_switching_position[0]-LC/2.0)*(mode_switching_position[0]-LC/2.0) + mode_switching_position[1]*mode_switching_position[1];
    const double tmp1 = LBs-LAs+d1s;
    const double from_tmp1 = (current_mode&1 ? 1.0 : -1.0)*sqrt(4.0*LBs*d1s - tmp1*tmp1);
    const double tmp2 = LBs-LAs+d2s;
    const double from_tmp2 = (current_mode&2 ? 1.0 : -1.0)*sqrt(4.0*LBs*d2s - tmp2*tmp2);
    memory->from_angle[0] = atan2(tmp1*mode_switching_position[1]+from_tmp1*(-mode_switching_position[0]-LC/2.0), tmp1*(mode_switching_position[0]+LC/2.0)+from_tmp1*mode_switching_position[1]);
    memory->from_angle[1] = atan2(tmp2*mode_switching_position[1]+from_tmp2*(-mode_switching_position[0]+LC/2.0), tmp2*(mode_switching_position[0]-LC/2.0)+from_tmp2*mode_switching_position[1]);
    memory->to_angle[0] = memory->from_angle[0];
    memory->to_angle[1] = memory->from_angle[1];
    memory->mode_switching_position[0] = mode_switching_position[0];
    memory->mode_switching_position[1] = mode_switching_position[1];
    memory->switching_arm = switching_arm;
    memory->mode_switching = 1;
  } else if (current_mode != target_mode && sqrt((load_xy[0]-mode_switching_position[0])*(load_xy[0]-mode_switching_position[0]) + (load_xy[1]-mode_switching_position[1])*(load_xy[1]-mode_switching_position[1])) < mode_switching_position_threshold) {
    switching_arm = (current_mode ^ target_mode) - 1; // 0: left arm 1: right arm
    if (fabs(angular_velocity[switching_arm]) < mode_switching_angular_speed_threshold) {
      memory->from = current_mode;
      memory->to = target_mode;
      const double d1s = (load_xy[0]+LC/2.0)*(load_xy[0]+LC/2.0) + load_xy[1]*load_xy[1];
      const double d2s = (load_xy[0]-LC/2.0)*(load_xy[0]-LC/2.0) + load_xy[1]*load_xy[1];
      const double tmp1 = LBs-LAs+d1s;
      const double from_tmp1 = (current_mode&1 ? 1.0 : -1.0)*sqrt(4.0*LBs*d1s - tmp1*tmp1);
      const double to_tmp1 = (target_mode&1 ? 1.0 : -1.0)*sqrt(4.0*LBs*d1s - tmp1*tmp1);
      const double tmp2 = LBs-LAs+d2s;
      const double from_tmp2 = (current_mode&2 ? 1.0 : -1.0)*sqrt(4.0*LBs*d2s - tmp2*tmp2);
      const double to_tmp2 = (target_mode&2 ? 1.0 : -1.0)*sqrt(4.0*LBs*d2s - tmp2*tmp2);
      memory->from_angle[0] = atan2(tmp1*load_xy[1]+from_tmp1*(-load_xy[0]-LC/2.0), tmp1*(load_xy[0]+LC/2.0)+from_tmp1*load_xy[1]);
      memory->from_angle[1] = atan2(tmp2*load_xy[1]+from_tmp2*(-load_xy[0]+LC/2.0), tmp2*(load_xy[0]-LC/2.0)+from_tmp2*load_xy[1]);
      memory->to_angle[0] = atan2(tmp1*load_xy[1]+to_tmp1*(-load_xy[0]-LC/2.0), tmp1*(load_xy[0]+LC/2.0)+to_tmp1*load_xy[1]);
      memory->to_angle[1] = atan2(tmp2*load_xy[1]+to_tmp2*(-load_xy[0]+LC/2.0), tmp2*(load_xy[0]-LC/2.0)+to_tmp2*load_xy[1]);
      memory->mode_switching_position[0] = mode_switching_position[0];
      memory->mode_switching_position[1] = mode_switching_position[1];
      memory->switching_arm = switching_arm;
      memory->mode_switching = 1;
    }
  }

  double nav_target[2];
  if (current_mode == target_mode) {
    sdf_pick_direction(nav_target, load_xy, target_xy, sdf_threshold, current_mode);
  } else {
    sdf_pick_direction(nav_target, load_xy, mode_switching_position, sdf_threshold, current_mode);
  }
  virtual_force(fv, load_xy, load_velocity, nav_target);
  alpha[0] = (magvs+angular_velocity[0]*vcrossr1)/r1crossr1p + vdotr1p*(vcrossr1+angular_velocity[0]*(r1dotr1p+LB*LB))/(r1crossr1p*r1crossr1p);
  alpha[1] = (magvs+angular_velocity[1]*vcrossr2)/r2crossr2p + vdotr2p*(vcrossr2+angular_velocity[1]*(r2dotr2p+LB*LB))/(r2crossr2p*r2crossr2p);
  goto torque_output;

mode_switching:
  double I_left, I_right;
  double kd_left, kd_right;
  if (memory->switching_arm == 0) { // left arm is stiff
    alpha[0] = 0.0;
    alpha[1] = (magvs+angular_velocity[1]*vcrossr2)/r2crossr2p + vdotr2p*(vcrossr2+angular_velocity[1]*(r2dotr2p+LB*LB))/(r2crossr2p*r2crossr2p);
    I_left = IM;
    I_right = IM + MLoad*(J[1]*J[1]+J[3]*J[3]);
  } else { // right arm is stiff
    alpha[0] = (magvs+angular_velocity[0]*vcrossr1)/r1crossr1p + vdotr1p*(vcrossr1+angular_velocity[0]*(r1dotr1p+LB*LB))/(r1crossr1p*r1crossr1p);
    alpha[1] = 0.0;
    I_left = IM + MLoad*(J[0]*J[0]+J[2]*J[2]);
    I_right = IM;
  }
  kd_left = 2.0*sqrt(mode_switching_kp*I_left);
  kd_right = 2.0*sqrt(mode_switching_kp*I_right);
  unsigned int mode;
  double *target_angle;
  if (target_mode == memory->to) {
    target_angle = memory->to_angle;
    mode = memory->to;
  } else if (target_mode == memory->from) {
    target_angle = memory->from_angle;
    mode = memory->from;
  } else if ((target_mode ^ memory->to) < 3) {
    target_angle = memory->to_angle;
    mode = memory->to;
  } else {
    target_angle = memory->from_angle;
    mode = memory->from;
  }
  double angle_err[2] = {wrap_angle(target_angle[0]-angle[0]), wrap_angle(target_angle[1]-angle[1])};
  torque_mode_switching[0] = mode_switching_kp*angle_err[0] - kd_left*angular_velocity[0];
  torque_mode_switching[1] = mode_switching_kp*angle_err[1] - kd_right*angular_velocity[1];
  const double distance = sqrt((load_xy[0]-memory->mode_switching_position[0])*(load_xy[0]-memory->mode_switching_position[0]) + (load_xy[1]-memory->mode_switching_position[1])*(load_xy[1]-memory->mode_switching_position[1]));
  virtual_force(fv, load_xy, load_velocity, current_mode == mode ? memory->mode_switching_position : load_xy);
  if (current_mode == mode && distance < mode_switching_position_threshold) {
    memory->mode_switching = 0;
  }
  goto torque_output;

torque_output:
  torque_fv[0] = J[0]*fv[0] + J[2]*fv[1];
  torque_fv[1] = J[1]*fv[0] + J[3]*fv[1];
  if (memory->mode_switching) {
    torque[0] = torque_fc[0] + torque_fv[0] + alpha[0]*IM + torque_mode_switching[0];
    torque[1] = torque_fc[1] + torque_fv[1] + alpha[1]*IM + torque_mode_switching[1];
  } else {
    torque[0] = torque_fc[0] + torque_fv[0] + alpha[0]*IM;
    torque[1] = torque_fc[1] + torque_fv[1] + alpha[1]*IM;
  }
  return;
halt:
  torque[0] = -100.0*angular_velocity[0];
  torque[1] = -100.0*angular_velocity[1];
}

void virtual_force_0(double *force, const double *load_xy, const double *load_velocity, const double *target_xy)
{
  const double velocity_cap = 1.0; // m/s
  const double kp = 40.0;
  const double kd_radial = 30.0;
  const double kd_tangential = 50.0;
  const double err[2] = {target_xy[0]-load_xy[0], target_xy[1]-load_xy[1]};
  const double magerr = sqrt(err[0]*err[0] + err[1]*err[1]);
  if (magerr > 1e-3) {
    const double err_dir[2] = {err[0]/magerr, err[1]/magerr};
    const double v_radialdoterr_dir = load_velocity[0]*err_dir[0] + load_velocity[1]*err_dir[1];
    const double v_radial[2] = {err_dir[0]*v_radialdoterr_dir, err_dir[1]*v_radialdoterr_dir};
    const double v_tangential[2] = {load_velocity[0]-v_radial[0], load_velocity[1]-v_radial[1]};
    double magforce_radial = magerr*kp - v_radialdoterr_dir*kd_radial;
    double force_radial[2];
    if (magforce_radial > 0.0 && v_radialdoterr_dir > velocity_cap) {
      force_radial[0] = 0.0;
      force_radial[1] = 0.0;
    } else {
      force_radial[0] = magforce_radial * err_dir[0];
      force_radial[1] = magforce_radial * err_dir[1];
    }
    force[0] = force_radial[0] - v_tangential[0]*kd_tangential;
    force[1] = force_radial[1] - v_tangential[1]*kd_tangential;
  } else {
    force[0] = err[0]*kp - load_velocity[0]*kd_radial;
    force[1] = err[1]*kp - load_velocity[1]*kd_radial;
  }
}

void draw_fb_state(fb_setup *fb, Vector2 origin, double display_scale)
{
  Vector2 load = {origin.x + display_scale*fb->state[6], origin.y - display_scale*fb->state[7]};
  Vector2 bar_lb[2], bar_la[2], bar_ra[2], bar_rb[2];
  double LA = fb->LA;
  double LB = fb->LB;
  double LC = fb->LC;
  double *w = fb->state;
  bar_lb[0].x = origin.x + display_scale*(w[0] - cos(w[2])*LB/2.0);
  bar_lb[0].y = origin.y - display_scale*(w[1] - sin(w[2])*LB/2.0);
  bar_lb[1].x = origin.x + display_scale*(w[0] + cos(w[2])*LB/2.0);
  bar_lb[1].y = origin.y - display_scale*(w[1] + sin(w[2])*LB/2.0);
  bar_la[0].x = origin.x + display_scale*(w[3] - cos(w[5])*LA/2.0);
  bar_la[0].y = origin.y - display_scale*(w[4] - sin(w[5])*LA/2.0);
  bar_la[1].x = origin.x + display_scale*(w[3] + cos(w[5])*LA/2.0);
  bar_la[1].y = origin.y - display_scale*(w[4] + sin(w[5])*LA/2.0);
  bar_ra[0].x = origin.x + display_scale*(w[8] - cos(w[10])*LA/2.0);
  bar_ra[0].y = origin.y - display_scale*(w[9] - sin(w[10])*LA/2.0);
  bar_ra[1].x = origin.x + display_scale*(w[8] + cos(w[10])*LA/2.0);
  bar_ra[1].y = origin.y - display_scale*(w[9] + sin(w[10])*LA/2.0);
  bar_rb[0].x = origin.x + display_scale*(w[11] - cos(w[13])*LB/2.0);
  bar_rb[0].y = origin.y - display_scale*(w[12] - sin(w[13])*LB/2.0);
  bar_rb[1].x = origin.x + display_scale*(w[11] + cos(w[13])*LB/2.0);
  bar_rb[1].y = origin.y - display_scale*(w[12] + sin(w[13])*LB/2.0);
  DrawCircleLinesV((Vector2){origin.x - display_scale*LC/2.0, origin.y}, display_scale*fabs(LA-LB), BLUE);
  DrawCircleLinesV((Vector2){origin.x + display_scale*LC/2.0, origin.y}, display_scale*fabs(LA-LB), BLUE);
  DrawCircleV(load, 10.0, GRAY);
  DrawLineV(bar_lb[0], bar_lb[1], YELLOW);
  DrawLineV(bar_la[0], bar_la[1], YELLOW);
  DrawLineV(bar_ra[0], bar_ra[1], YELLOW);
  DrawLineV(bar_rb[0], bar_rb[1], YELLOW);
  DrawRingLines((Vector2){origin.x + display_scale*LC/2.0, origin.y},
    display_scale*(LA+LB), display_scale*(LA+LB),
    180.0-acos(0.5*LC/(LA+LB))*180.0/PI, 180.0+acos(0.5*LC/(LA+LB))*180.0/PI, 31, BLUE);
  DrawRingLines((Vector2){origin.x - display_scale*LC/2.0, origin.y},
    display_scale*(LA+LB), display_scale*(LA+LB),
    -acos(0.5*LC/(LA+LB))*180.0/PI, acos(0.5*LC/(LA+LB))*180.0/PI, 31, BLUE);
}

int main(void)
{
  const double LA = 1.0;
  const double LB = 0.8;
  const double LC = 0.7;
  const double MA = 0.2;
  const double MB = 1.0;
  const double MLoad = 1.0;
  const double IA = 0.1;
  const double IB = 1.0;

  // const double MA = 0.01;
  // const double MB = 0.01;
  // const double MLoad = 1.0;
  // const double IA = 0.01;
  // const double IB = 0.01;
  fb_setup *fb = make_fb_setup(LA, LB, LC, MA, IA, MB, IB, MLoad);
  fb_setup_init_xy(fb, 0.0, 1.0, 2);
  // fb_setup_init_ang(fb, 0.0, 0.0, 1);

  // fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_naive, .controller_data=NULL, .target_xy={0.0, 1.5}, .running=1};
  // double display_data[8];
  // fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_test3, .controller_data=display_target_positions, .target_xy={0.0, 1.5}, .running=1};
  controller_memory controller_state = {.mode_switching = 0};
  // fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_test5, .controller_data=&controller_state, .target_xy={0.0, 1.5}, .running=0};
  fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_0, .memory=&controller_state, .target_xy={0.0, 1.5},
    .virtual_force=virtual_force_0, .compensation_force=NULL, .running=0};
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, fb_sim_runner, &simulation);

  SetTraceLogLevel(LOG_ERROR);
  InitWindow(WINDOW_W, WINDOW_H, "fb_util test");
  SetTargetFPS(60);
  SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);
  while (!WindowShouldClose()) {
    Vector2 origin = {WINDOW_W/2.0, WINDOW_H/2.0};
    double display_scale = 180.0;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      Vector2 mouse_pos = GetMousePosition();
      double new_x = (mouse_pos.x-origin.x)/display_scale;
      double new_y = -(mouse_pos.y-origin.y)/display_scale;
      if (fb_setup_is_xy_reachable(fb, new_x, new_y)) {
        simulation.target_xy[0] = new_x;
        simulation.target_xy[1] = new_y;
      }
    }
    if (IsKeyPressed(KEY_SPACE)) {
      simulation.running = !simulation.running;
    }
    BeginDrawing();
    ClearBackground((Color){0, 0, 0, 255});
    DrawCircleLinesV((Vector2){origin.x+display_scale*simulation.target_xy[0], origin.y-display_scale*simulation.target_xy[1]},
      10.0, GREEN);
    draw_fb_state(fb, origin, display_scale);
    DrawText(fb->assembly_mode==1 ? "ASSEMBLY MODE: +" : "ASSEMBLY MODE: -", 200, 60, 20, WHITE);
    DrawText((const char *[4]){"WORKING MODE: --", "WORKING MODE: +-", "WORKING MODE: -+", "WORKING MODE: ++"}[fb->working_mode], 200, 80, 20, WHITE);
    // double J[4];
    // fb_setup_jacobian(J, fb);
    // printf("%f\n", J[0]*J[3]-J[2]*J[1]);
    char simulation_time[30];
    snprintf(simulation_time, 30, "TIME: %.3f",simulation.fb->sim_time);
    DrawText(simulation_time, 200, 100, 20, WHITE);
    // DrawFPS(0, 0);
    EndDrawing();
  }
  pthread_cancel(thread_id);
  return 0;
}