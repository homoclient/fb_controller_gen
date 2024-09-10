#include <stdio.h>
#include <math.h>
// #include <unistd.h>
#include "usleep.h"
#include <pthread.h>
#include <assert.h>
#include "fb_util.h"
#include "raylib.h"

// gcc -lm -lopengl32 -pthread fb_util.c simple_linalg.c fb_sim_2.c usleep.c raylib.dll
// gcc -lm -lopengl32 -pthread fb_util.c simple_linalg.c fb_sim_2.c raylib.dll

#define WINDOW_W 1024
#define WINDOW_H 768

typedef struct {
  fb_setup *fb;
  void (*controller)(double *, fb_setup *, const double *, void *);
  void *controller_data;
  double target_xy[2];
  double dt;
  int running;
} fb_sim;

void *fb_sim_runner(void *p)
{
  fb_setup *fb = ((fb_sim *)p)->fb;
  void (*fb_controller)(double *, fb_setup *, const double *, void *) = ((fb_sim *)p)->controller;
  double *target = ((fb_sim *)p)->target_xy;
  while (1) {
    if (((fb_sim *)p)->running) {
      fb_controller(fb->torque, fb, target, ((fb_sim *)p)->controller_data);
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

// double sdf_lut_sample(double x, double y, unsigned int mode)
// {
//   assert(fabs(x)<SDF_LUT_W*SDF_LUT_GRID_LENGTH/2.0 && fabs(y)<SDF_LUT_H*SDF_LUT_GRID_LENGTH/2.0);
//   unsigned int grid_x, grid_y;
//   switch (mode) {
//     case 0:
//       grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
//     case 1:
//       grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[1][grid_x+grid_y*SDF_LUT_W];
//     case 2:
//       grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[2][grid_x+grid_y*SDF_LUT_W];
//     case 3:
//       grid_x = (unsigned int)(-x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
//     case 4:
//       grid_x = (unsigned int)(-x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
//     case 5:
//       grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[2][grid_x+grid_y*SDF_LUT_W];
//     case 6:
//       grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[1][grid_x+grid_y*SDF_LUT_W];
//     case 7:
//       grid_x = (unsigned int)(x/SDF_LUT_GRID_LENGTH + SDF_LUT_X_ORIGIN - 0.5);
//       grid_y = (unsigned int)(-y/SDF_LUT_GRID_LENGTH + SDF_LUT_Y_ORIGIN - 0.5);
//       return SDF_LUT_GRID_LENGTH*sdf_lut[0][grid_x+grid_y*SDF_LUT_W];
//   }
//   return 0.0; // not reached
// }

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

double working_mode_border_position(double *res, const double *from, const double *to, unsigned int from_mode, unsigned int to_mode, double LA, double LB, double LC)
{
  const double safe_distance = 0.03;
  const double safety_factor_inner = 0.4;
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

typedef struct {
  int mode_crossing;
  double target_ang[2];
} controller_memory;

void controller_test5(double *torque, fb_setup *fb, const double *target, void *data)
{
  controller_memory *controller_state = (controller_memory *)data;
  const double kp = 40.0;
  const double kd = 15.0;
  const double *current_xy = fb->state+6;
  const double current_ang[2] = {fb->state[2], fb->state[13]};
  const double *current_v = fb->state+(6+14);
  const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
// halt if assembly mode is ambiguous
  const double assembly_mode_ambiguous_threshold = 5e-3;
  const double r1px = current_xy[0] - LB*cos(current_ang[0]) + LC/2.0;
  const double r1py = current_xy[1] - LB*sin(current_ang[0]);
  const double r2px = current_xy[0] - LB*cos(current_ang[1]) - LC/2.0;
  const double r2py = current_xy[1] - LB*sin(current_ang[1]);
  const double r1pcorssr2p = r1px*r2py - r1py*r2px;
  const double sgl2_condition = fabs(r1pcorssr2p/(LA*LA));
  if (sgl2_condition < assembly_mode_ambiguous_threshold) {
    printf("halt for ambiguous assembly mode\n");
    goto halt;
  }
  const double dr1r2s = LB*LB*((cos(current_ang[0])-cos(current_ang[1]))*(cos(current_ang[0])-cos(current_ang[1]))+(sin(current_ang[0])-sin(current_ang[1]))*(sin(current_ang[0])-sin(current_ang[1]))) - 2*LB*LC*(cos(current_ang[0])-cos(current_ang[1])) + LC*LC;
  const double sgl2_tmp = PI*LB*cos(PI*dr1r2s/(4*LA*LA))/(2.0*LA*LA);
  const double am_grad1 = sgl2_tmp*(LB*sin(current_ang[0]-current_ang[1]) + LC*sin(current_ang[0]));
  const double am_grad2 = -sgl2_tmp*(LB*sin(current_ang[0]-current_ang[1]) + LC*sin(current_ang[1]));
  if (controller_state->mode_crossing) goto mode_crossing;

  double J[4];
  fb_setup_jacobian(J, fb);
  const double sgl2_avoiding_direction[2] = {J[0]*am_grad1 + J[1]*am_grad2, J[2]*am_grad1 + J[3]*am_grad2};
  const double user_safe_distance = 0.1;
  const double sdf_threshold = 0.02;
  const unsigned int current_mode = fb_setup_tell_mode(fb);
  unsigned int target_mode;
  double actual_target[2];
  double mode_crossing_position[2];
  if (sdf_lut_sample(target[0], target[1], current_mode) > user_safe_distance) {
    target_mode = current_mode;
    // working_mode_border_position(mode_crossing_position, current_xy, current_xy, current_mode, current_mode, LA, LB, LC);
    goto navigation;
  }
  int target_in_adjacent_modes = 0;
  double mode_switching_distance_1;
  double temp_target_1[2];
  const unsigned int adjacent_mode_1 = current_mode ^ 1;
  if (sdf_lut_sample(target[0], target[1], adjacent_mode_1) > user_safe_distance) {
    target_in_adjacent_modes |= 1;
    mode_switching_distance_1 = working_mode_border_position(temp_target_1, current_xy, target, current_mode, adjacent_mode_1, LA, LB, LC);
  }
  double mode_switching_distance_2;
  double temp_target_2[2];
  const unsigned int adjacent_mode_2 = current_mode ^ 2;
  if (sdf_lut_sample(target[0], target[1], adjacent_mode_2) > user_safe_distance) {
    target_in_adjacent_modes |= 2;
    mode_switching_distance_2 = working_mode_border_position(temp_target_2, current_xy, target, current_mode, adjacent_mode_2, LA, LB, LC);
  }
  switch (target_in_adjacent_modes) {
    case 0: 
      printf("halt for having no way to reach target\n");
      goto halt;
    case 1:
      mode_crossing_position[0] = temp_target_1[0]; mode_crossing_position[1] = temp_target_1[1];
      target_mode = adjacent_mode_1;
    break;
    case 2:
      mode_crossing_position[0] = temp_target_2[0]; mode_crossing_position[1] = temp_target_2[1];
      target_mode = adjacent_mode_2;
    break;
    case 3:
      if (mode_switching_distance_1 < mode_switching_distance_2) {
        mode_crossing_position[0] = temp_target_1[0]; mode_crossing_position[1] = temp_target_1[1];
        target_mode = adjacent_mode_1;
      } else {
        mode_crossing_position[0] = temp_target_2[0]; mode_crossing_position[1] = temp_target_2[1];
        target_mode = adjacent_mode_2;
      }
    break;
    default: // not reached
      assert(0);
  }

navigation:
  const double distance_to_mode_crossing_position = sqrt((current_xy[0]-mode_crossing_position[0])*(current_xy[0]-mode_crossing_position[0]) + (current_xy[1]-mode_crossing_position[1])*(current_xy[1]-mode_crossing_position[1]));
  double nav_target[2];
  if (current_mode != target_mode) {
    sdf_pick_direction(nav_target, current_xy, mode_crossing_position, sdf_threshold, current_mode);
  } else {
    sdf_pick_direction(nav_target, current_xy, target, sdf_threshold, current_mode);
  }
  double coriolis_accel[2];
  fb_setup_coriolis_accel(coriolis_accel, fb);
  const double err_xy[2] = {nav_target[0]-current_xy[0], nav_target[1]-current_xy[1]};
  const double end_force[2] = {kp*err_xy[0] - kd*current_v[0], kp*err_xy[1] - kd*current_v[1]};
  double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
  torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
  torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

  const double mag_current_v = sqrt(current_v[0]*current_v[0] + current_v[1]*current_v[1]);
  if (current_mode != target_mode && distance_to_mode_crossing_position < 0.05 && mag_current_v < 0.5) {
    controller_state->mode_crossing = 1;
    double new_ang[2];
    fb_setup_ik(controller_state->target_ang, fb, mode_crossing_position[0], mode_crossing_position[1], target_mode&3);
  }
  goto torque_limiting;

mode_crossing:
  const double kp_mode_crossing = 30.0;
  const double kd_mode_crossing = 10.0;
  const double err_ang[2] = {
    wrap_angle(controller_state->target_ang[0] - current_ang[0]),
    wrap_angle(controller_state->target_ang[1] - current_ang[1])
  };
  torque[0] = kp_mode_crossing*err_ang[0] - kd_mode_crossing*current_w[0];
  torque[1] = kp_mode_crossing*err_ang[1] - kd_mode_crossing*current_w[1];
  const double mag_err_ang = sqrt(err_ang[0]*err_ang[0] + err_ang[1]*err_ang[1]);
  if (mag_err_ang < 0.05) {
    controller_state->mode_crossing = 0;
  }
  goto torque_limiting;

torque_limiting:
  const double torque_limit = 100.0;
  const double torque_mag = sqrt(torque[0]*torque[0] + torque[1]*torque[1]);
  if (torque_mag > torque_limit) {
    torque[0] *= torque_limit/torque_mag;
    torque[1] *= torque_limit/torque_mag;
  }
  return;
halt:
  const double kd_halt = 100.0;
  torque[0] = -kd_halt*current_w[0];
  torque[1] = -kd_halt*current_w[1];
}

// void controller_test4(double *torque, fb_setup *fb, const double *target, void *data)
// {
//   double *display = (double *)data;
//   if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
//   const double kp = 80.0;
//   const double kd = 50.0;
//   const double *current_xy = fb->state+6;
//   const double current_ang[2] = {fb->state[2], fb->state[13]};
//   const double *current_v = fb->state+(6+14);
//   const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
//   const double LA = fb->LA;
//   const double LB = fb->LB;
//   const double LC = fb->LC;
// // halt if assembly mode is ambiguous
//   const double assembly_mode_ambiguous_threshold = 5e-3;
//   const double r1px = current_xy[0] - LB*cos(current_ang[0]) + LC/2.0;
//   const double r1py = current_xy[1] - LB*sin(current_ang[0]);
//   const double r2px = current_xy[0] - LB*cos(current_ang[1]) - LC/2.0;
//   const double r2py = current_xy[1] - LB*sin(current_ang[1]);
//   const double r1pcorssr2p = r1px*r2py - r1py*r2px;
//   const double sgl2_condition = fabs(r1pcorssr2p/(LA*LA));
//   if (sgl2_condition < assembly_mode_ambiguous_threshold) {
//     printf("halt for ambiguous assembly mode\n");
//     goto halt;
//   }
//   double J[4];
//   fb_setup_jacobian(J, fb);
//   const double dr1r2s = LB*LB*((cos(current_ang[0])-cos(current_ang[1]))*(cos(current_ang[0])-cos(current_ang[1]))+(sin(current_ang[0])-sin(current_ang[1]))*(sin(current_ang[0])-sin(current_ang[1]))) - 2*LB*LC*(cos(current_ang[0])-cos(current_ang[1])) + LC*LC;
//   const double sgl2_tmp = PI*LB*cos(PI*dr1r2s/(4*LA*LA))/(2.0*LA*LA);
//   const double am_grad1 = sgl2_tmp*(LB*sin(current_ang[0]-current_ang[1]) + LC*sin(current_ang[0]));
//   const double am_grad2 = -sgl2_tmp*(LB*sin(current_ang[0]-current_ang[1]) + LC*sin(current_ang[1]));
//   const double sgl2_avoiding_direction[2] = {J[0]*am_grad1 + J[1]*am_grad2, J[2]*am_grad1 + J[3]*am_grad2};
//   double mode_handling_direction[2];
//   display[0] = J[0];
//   display[1] = J[2];
//   display[2] = J[1];
//   display[3] = J[3];
//   // display[4] = sgl2_avoiding_direction[0];
//   // display[5] = sgl2_avoiding_direction[1];
//   const double user_safe_distance = 0.1;
//   const double mode_handling_distance = 0.1;
//   const double mode_crossing_distance = 0.08;
//   const double sdf_threshold = 0.02;
//   const unsigned int current_mode = fb_setup_tell_mode(fb);
//   unsigned int target_mode;
//   double actual_target[2];
//   double mode_crossing_position[2];
//   if (sdf_lut_sample(target[0], target[1], current_mode) > user_safe_distance) {
//     target_mode = current_mode;
//     working_mode_border_position(mode_crossing_position, current_xy, current_xy, current_mode, current_mode, LA, LB, LC);
//     goto navigation;
//   }
//   int target_in_adjacent_modes = 0;
//   double mode_switching_distance_1;
//   double temp_target_1[2];
//   const unsigned int adjacent_mode_1 = current_mode ^ 1;
//   if (sdf_lut_sample(target[0], target[1], adjacent_mode_1) > user_safe_distance) {
//     target_in_adjacent_modes |= 1;
//     mode_switching_distance_1 = working_mode_border_position(temp_target_1, current_xy, target, current_mode, adjacent_mode_1, LA, LB, LC);
//   }
//   double mode_switching_distance_2;
//   double temp_target_2[2];
//   const unsigned int adjacent_mode_2 = current_mode ^ 2;
//   if (sdf_lut_sample(target[0], target[1], adjacent_mode_2) > user_safe_distance) {
//     target_in_adjacent_modes |= 2;
//     mode_switching_distance_2 = working_mode_border_position(temp_target_2, current_xy, target, current_mode, adjacent_mode_2, LA, LB, LC);
//   }
//   switch (target_in_adjacent_modes) {
//     case 0: 
//       printf("halt for having no way to reach target\n");
//       goto halt;
//     case 1:
//       mode_crossing_position[0] = temp_target_1[0]; mode_crossing_position[1] = temp_target_1[1];
//       target_mode = adjacent_mode_1;
//     break;
//     case 2:
//       mode_crossing_position[0] = temp_target_2[0]; mode_crossing_position[1] = temp_target_2[1];
//       target_mode = adjacent_mode_2;
//     break;
//     case 3:
//       if (mode_switching_distance_1 < mode_switching_distance_2) {
//         mode_crossing_position[0] = temp_target_1[0]; mode_crossing_position[1] = temp_target_1[1];
//         target_mode = adjacent_mode_1;
//       } else {
//         mode_crossing_position[0] = temp_target_2[0]; mode_crossing_position[1] = temp_target_2[1];
//         target_mode = adjacent_mode_2;
//       }
//     break;
//     default: // not reached
//       assert(0);
//   }

// navigation:
//   const double inner_radius = fabs(LA-LB);
//   const double outer_radius = LA+LB;
//   const double r_left = sqrt((current_xy[0]+LC/2.0)*(current_xy[0]+LC/2.0)+current_xy[1]*current_xy[1]);
//   const double r_right = sqrt((current_xy[0]-LC/2.0)*(current_xy[0]-LC/2.0)+current_xy[1]*current_xy[1]);
//   const double distance_left_inner = fabs(r_left-inner_radius);
//   const double distance_left_outer = fabs(r_left-outer_radius);
//   const double distance_right_inner = fabs(r_right-inner_radius);
//   const double distance_right_outer = fabs(r_right-outer_radius);
//   do {
//     double tmp[2] = {0.0, 0.0};
//     if (distance_left_inner < mode_handling_distance) {
//       wm_crossing_direction(tmp, J, current_mode, target_mode, 0, 0);
//     } else if (distance_left_outer < mode_handling_distance) {
//       wm_crossing_direction(tmp, J, current_mode, target_mode, 0, 1);
//     } else if (distance_right_inner < mode_handling_distance) {
//       wm_crossing_direction(tmp, J, current_mode, target_mode, 1, 0);
//     } else if (distance_right_outer < mode_handling_distance) {
//       wm_crossing_direction(tmp, J, current_mode, target_mode, 1, 1);
//     }
//     mode_handling_direction[0] = tmp[0];
//     mode_handling_direction[1] = tmp[1];
//     display[4] = mode_handling_direction[0];
//     display[5] = mode_handling_direction[1];
//   } while (0);
  
//   const double distance_to_mode_crossing_position = sqrt((current_xy[0]-mode_crossing_position[0])*(current_xy[0]-mode_crossing_position[0]) + (current_xy[1]-mode_crossing_position[1])*(current_xy[1]-mode_crossing_position[1]));
//   const double mode_handling_weight = 1.0 - fmin(1.0, (distance_to_mode_crossing_position - mode_crossing_distance)/(mode_handling_distance - mode_crossing_distance));
//   const int position_error_enabled = distance_to_mode_crossing_position > mode_crossing_distance;
//   double nav_target[2];
//   if (position_error_enabled) {
//     if (current_mode != target_mode) {
//       sdf_pick_direction(nav_target, current_xy, mode_crossing_position, sdf_threshold, current_mode);
//     } else {
//       sdf_pick_direction(nav_target, current_xy, target, sdf_threshold, current_mode);
//     }
//   } else {
//     nav_target[0] = current_xy[0];
//     nav_target[1] = current_xy[1];
//   }
//   display[6] = nav_target[0];
//   display[7] = nav_target[1];
//   // display[6] = mode_crossing_position[0];
//   // display[7] = mode_crossing_position[1];
//   double coriolis_accel[2];
//   fb_setup_coriolis_accel(coriolis_accel, fb);
//   // if (!position_error_enabled) {
//   //   printf("%.3f, %.3f\n", coriolis_accel[0], coriolis_accel[1]);
//   // }
//   // if (position_error_enabled) {
//   //   fb_setup_coriolis_accel(coriolis_accel, fb);
//   // } else {
//   //   coriolis_accel[0] = 0.0;
//   //   coriolis_accel[1] = 0.0;
//   // }
//   const double err_xy[2] = {position_error_enabled ? nav_target[0]-current_xy[0] : 0.0, position_error_enabled ? nav_target[1]-current_xy[1] : 0.0};
//   const double end_force[2] = {kp*err_xy[0] + kd*(mode_handling_direction[0]*mode_handling_weight - current_v[0]), kp*err_xy[1] + kd*(mode_handling_direction[1]*mode_handling_weight - current_v[1])};
//   double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
//   torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
//   torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

// // torque limiting
//   const double torque_limit = 100.0;
//   const double torque_mag = sqrt(torque[0]*torque[0] + torque[1]*torque[1]);
//   if (torque_mag > torque_limit) {
//     torque[0] *= torque_limit/torque_mag;
//     torque[1] *= torque_limit/torque_mag;
//   }
//   return;
// halt:
//   const double kd_halt = 100.0;
//   torque[0] = -kd_halt*current_w[0];
//   torque[1] = -kd_halt*current_w[1];
// }

// void controller_test3(double *torque, fb_setup *fb, const double *target, void *data)
// {
//   double *display_data = (double *)data;
//   if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
//   const double kp = 80.0;
//   const double kd = 40;
//   const double *current_xy = fb->state+6;
//   const double current_ang[2] = {fb->state[2], fb->state[13]};
//   const double *current_v = fb->state+(6+14);
//   const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
//   const double LA = fb->LA;
//   const double LB = fb->LB;
//   const double LC = fb->LC;
// // halt if assembly mode is ambiguous
//   const double assembly_mode_ambiguous_threshold = 5e-3;
//   const double r1px = current_xy[0] - LB*cos(current_ang[0]) + LC/2.0;
//   const double r1py = current_xy[1] - LB*sin(current_ang[0]);
//   const double r2px = current_xy[0] - LB*cos(current_ang[1]) - LC/2.0;
//   const double r2py = current_xy[1] - LB*sin(current_ang[1]);
//   const double r1pcorssr2p = r1px*r2py - r1py*r2px;
//   if (fabs(r1pcorssr2p/(LA*LA)) < assembly_mode_ambiguous_threshold) {
//     printf("halt for ambiguous assembly mode\n");
//     goto halt;
//   }
//   const double user_safe_distance = 0.08;
//   const double mode_crossing_distance = 0.06;
//   const double sdf_threshold_mode_corssing = 0.02;
//   const double sdf_threshold_normal = 0.07;
//   const unsigned int current_mode = fb_setup_tell_mode(fb);
//   unsigned int target_mode;
//   double actual_target[2];
//   if (sdf_lut_sample(target[0], target[1], current_mode) > user_safe_distance) {
//     target_mode = current_mode;
//     actual_target[0] = target[0];
//     actual_target[1] = target[1];
//     goto navigation;
//   }
//   int target_in_adjacent_modes = 0;
//   double mode_switching_distance_1;
//   double temp_target_1[2];
//   const unsigned int adjacent_mode_1 = current_mode ^ 1;
//   if (sdf_lut_sample(target[0], target[1], adjacent_mode_1) > user_safe_distance) {
//     target_in_adjacent_modes |= 1;
//     mode_switching_distance_1 = working_mode_border_position(temp_target_1, current_xy, target, current_mode, adjacent_mode_1, LA, LB, LC);
//   }
//   double mode_switching_distance_2;
//   double temp_target_2[2];
//   const unsigned int adjacent_mode_2 = current_mode ^ 2;
//   if (sdf_lut_sample(target[0], target[1], adjacent_mode_2) > user_safe_distance) {
//     target_in_adjacent_modes |= 2;
//     mode_switching_distance_2 = working_mode_border_position(temp_target_2, current_xy, target, current_mode, adjacent_mode_2, LA, LB, LC);
//   }
//   switch (target_in_adjacent_modes) {
//     case 0: 
//       printf("halt for having no way to reach target\n");
//       goto halt;
//     case 1:
//       actual_target[0] = temp_target_1[0]; actual_target[1] = temp_target_1[1];
//       target_mode = adjacent_mode_1;
//     break;
//     case 2:
//       actual_target[0] = temp_target_2[0]; actual_target[1] = temp_target_2[1];
//       target_mode = adjacent_mode_2;
//     break;
//     case 3:
//       if (mode_switching_distance_1 < mode_switching_distance_2) {
//         actual_target[0] = temp_target_1[0]; actual_target[1] = temp_target_1[1];
//         target_mode = adjacent_mode_1;
//       } else {
//         actual_target[0] = temp_target_2[0]; actual_target[1] = temp_target_2[1];
//         target_mode = adjacent_mode_2;
//       }
//     break;
//     default: // not reached
//       assert(0);
//   }
//   // printf("target mode: %u\n", target_mode);
//   navigation:
//   const double inner_radius = fabs(LA-LB);
//   const double outer_radius = LA+LB;
//   const double r_left = sqrt((current_xy[0]+LC/2.0)*(current_xy[0]+LC/2.0)+current_xy[1]*current_xy[1]);
//   const double r_right = sqrt((current_xy[0]-LC/2.0)*(current_xy[0]-LC/2.0)+current_xy[1]*current_xy[1]);
//   const double distance_left_inner = fabs(r_left-inner_radius);
//   const double distance_left_outer = fabs(r_left-outer_radius);
//   const double distance_right_inner = fabs(r_right-inner_radius);
//   const double distance_right_outer = fabs(r_right-outer_radius);
//   if (fabs(r1pcorssr2p/(LA*LA)) > 0.1) {
//     double direction[2];
//     double J[4];
//     fb_setup_jacobian(J, fb);
//     if (distance_left_inner < mode_crossing_distance) {
//       wm_crossing_direction(direction, J, current_mode, target_mode, 0, 0);
//       torque[0] = direction[0]*J[0]+direction[1]*J[2];
//       torque[1] = direction[0]*J[1]+direction[1]*J[3];
//       goto torque_limiting;
//     } else if (distance_left_outer < mode_crossing_distance) {
//       wm_crossing_direction(direction, J, current_mode, target_mode, 0, 1);
//       torque[0] = direction[0]*J[0]+direction[1]*J[2];
//       torque[1] = direction[0]*J[1]+direction[1]*J[3];
//       goto torque_limiting;
//     }
//     if (distance_right_inner < mode_crossing_distance) {
//       wm_crossing_direction(direction, J, current_mode, target_mode, 1, 0);
//       torque[0] = direction[0]*J[0]+direction[1]*J[2];
//       torque[1] = direction[0]*J[1]+direction[1]*J[3];
//       goto torque_limiting;
//     } else if (distance_right_outer < mode_crossing_distance) {
//       wm_crossing_direction(direction, J, current_mode, target_mode, 1, 1);
//       torque[0] = direction[0]*J[0]+direction[1]*J[2];
//       torque[1] = direction[0]*J[1]+direction[1]*J[3];
//       goto torque_limiting;
//     }
//   } else {
//     printf("fuck");
//     goto halt;
//   }
//   double nav_target[2];
//   sdf_pick_direction(nav_target, current_xy, actual_target, (current_mode == target_mode) ? sdf_threshold_normal : sdf_threshold_mode_corssing, current_mode);
//   display_data[0] = actual_target[0];
//   display_data[1] = actual_target[1];
//   display_data[2] = nav_target[0];
//   display_data[3] = nav_target[1];
//   double J[4];
//   fb_setup_jacobian(J, fb);
//   double coriolis_accel[2];
//   fb_setup_coriolis_accel(coriolis_accel, fb);
//   const double err_xy[2] = {nav_target[0]-current_xy[0], nav_target[1]-current_xy[1]};
//   const double end_force[2] = {kp*err_xy[0] - kd*current_v[0], kp*err_xy[1] - kd*current_v[1]};
//   double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
//   torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
//   torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

// torque_limiting:
//   const double torque_limit = 40.0;
//   const double torque_mag = sqrt(torque[0]*torque[0] + torque[1]*torque[1]);
//   if (torque_mag > torque_limit) {
//     torque[0] *= torque_limit/torque_mag;
//     torque[1] *= torque_limit/torque_mag;
//   }
//   return;
// halt:
//   const double kd_halt = 100.0;
//   torque[0] = -kd_halt*current_w[0];
//   torque[1] = -kd_halt*current_w[1];
// }

// void controller_test3(double *torque, fb_setup *fb, const double *target, void *data)
// {
//   double *display_data = (double *)data;
//   if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
//   const double kp = 80.0;
//   const double kd = 40;
//   const double *current_xy = fb->state+6;
//   const double current_ang[2] = {fb->state[2], fb->state[13]};
//   const double *current_v = fb->state+(6+14);
//   const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
//   const double LA = fb->LA;
//   const double LB = fb->LB;
//   const double LC = fb->LC;
// // halt if assembly mode is ambiguous
//   const double assembly_mode_ambiguous_threshold = 5e-3;
//   const double r1px = current_xy[0] - LB*cos(current_ang[0]) + LC/2.0;
//   const double r1py = current_xy[1] - LB*sin(current_ang[0]);
//   const double r2px = current_xy[0] - LB*cos(current_ang[1]) - LC/2.0;
//   const double r2py = current_xy[1] - LB*sin(current_ang[1]);
//   const double r1pcorssr2p = r1px*r2py - r1py*r2px;
//   if (fabs(r1pcorssr2p/(LA*LA)) < assembly_mode_ambiguous_threshold) {
//     printf("halt for ambiguous assembly mode\n");
//     goto halt;
//   }
//   const double user_safe_distance = 0.08;
//   const double mode_crossing_threshold = 0.1;
//   const double sdf_threshold = 0.03;
//   const unsigned int current_mode = fb_setup_tell_mode(fb);
//   unsigned int target_mode;
//   double actual_target[2];
//   if (sdf_lut_sample(target[0], target[1], current_mode) > user_safe_distance) {
//     target_mode = current_mode;
//     actual_target[0] = target[0];
//     actual_target[1] = target[1];
//     goto navigation;
//   }
//   int target_in_adjacent_modes = 0;
//   double mode_switching_distance_1;
//   double temp_target_1[2];
//   const unsigned int adjacent_mode_1 = current_mode ^ 1;
//   if (sdf_lut_sample(target[0], target[1], adjacent_mode_1) > user_safe_distance) {
//     target_in_adjacent_modes |= 1;
//     mode_switching_distance_1 = working_mode_border_position(temp_target_1, current_xy, target, current_mode, adjacent_mode_1, LA, LB, LC);
//   }
//   double mode_switching_distance_2;
//   double temp_target_2[2];
//   const unsigned int adjacent_mode_2 = current_mode ^ 2;
//   if (sdf_lut_sample(target[0], target[1], adjacent_mode_2) > user_safe_distance) {
//     target_in_adjacent_modes |= 2;
//     mode_switching_distance_2 = working_mode_border_position(temp_target_2, current_xy, target, current_mode, adjacent_mode_2, LA, LB, LC);
//   }
//   switch (target_in_adjacent_modes) {
//     case 0: 
//       printf("halt for having no way to reach target\n");
//       goto halt;
//     case 1:
//       actual_target[0] = temp_target_1[0]; actual_target[1] = temp_target_1[1];
//       target_mode = adjacent_mode_1;
//     break;
//     case 2:
//       actual_target[0] = temp_target_2[0]; actual_target[1] = temp_target_2[1];
//       target_mode = adjacent_mode_2;
//     break;
//     case 3:
//       if (mode_switching_distance_1 < mode_switching_distance_2) {
//         actual_target[0] = temp_target_1[0]; actual_target[1] = temp_target_1[1];
//         target_mode = adjacent_mode_1;
//       } else {
//         actual_target[0] = temp_target_2[0]; actual_target[1] = temp_target_2[1];
//         target_mode = adjacent_mode_2;
//       }
//     break;
//     default: // not reached
//       assert(0);
//   }
//   // printf("target mode: %u\n", target_mode);
//   navigation:
//   const double inner_radius = fabs(LA-LB);
//   const double outer_radius = LA+LB;
//   const double r_left = sqrt((current_xy[0]+LC/2.0)*(current_xy[0]+LC/2.0)+current_xy[1]*current_xy[1]);
//   const double r_right = sqrt((current_xy[0]-LC/2.0)*(current_xy[0]-LC/2.0)+current_xy[1]*current_xy[1]);
//   const double distance_left = fmin(fabs(r_left-inner_radius), fabs(r_left-outer_radius));
//   const double distance_right = fmin(fabs(r_right-inner_radius), fabs(r_right-outer_radius));
//   if (fmin(distance_left, distance_right) < (current_mode==target_mode ? 0.04 : mode_crossing_threshold)) {
//     double mode_handling_speed = 10.0;
//     double dir[2];
//     double err[2];
//     double frac = fabs(r1pcorssr2p/(LA*LA)) < 0.1 ? 0.0 : 0.8;
//     _mode_handling(dir, current_xy, current_ang, LA, LB, LC, target_mode, frac);
//     err[0] = dir[0]*mode_handling_speed - current_w[0];
//     err[1] = dir[1]*mode_handling_speed - current_w[1];
//     torque[0] = err[0]*5.0;
//     torque[1] = err[1]*5.0;
//     // printf("from %u to %u\n", current_mode, target_mode);
//     printf("%.2f %.2f\n", torque[0], torque[1]);
//     goto torque_limiting;
//   }
//   double nav_target[2];
//   sdf_pick_direction(nav_target, current_xy, actual_target, sdf_threshold, current_mode);
//   display_data[0] = actual_target[0];
//   display_data[1] = actual_target[1];
//   display_data[2] = nav_target[0];
//   display_data[3] = nav_target[1];
//   double J[4];
//   fb_setup_jacobian(J, fb);
//   double coriolis_accel[2];
//   fb_setup_coriolis_accel(coriolis_accel, fb);
//   const double err_xy[2] = {nav_target[0]-current_xy[0], nav_target[1]-current_xy[1]};
//   const double end_force[2] = {kp*err_xy[0] - kd*current_v[0], kp*err_xy[1] - kd*current_v[1]};
//   double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
//   torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
//   torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

// torque_limiting:
//   const double torque_limit = 20.0;
//   const double torque_mag = sqrt(torque[0]*torque[0] + torque[1]*torque[1]);
//   if (torque_mag > torque_limit) {
//     torque[0] *= torque_limit/torque_mag;
//     torque[1] *= torque_limit/torque_mag;
//   }
//   return;
// halt:
//   const double kd_halt = 100.0;
//   torque[0] = -kd_halt*current_w[0];
//   torque[1] = -kd_halt*current_w[1];
// }

// void controller_test2(double *torque, fb_setup *fb, const double *target, void *data)
// {
//   const double kp = 100.0;
//   const double kd = 50;
//   const double *current_xy = fb->state+6;
//   const double current_ang[2] = {fb->state[2], fb->state[13]};
//   const double *current_v = fb->state+(6+14);
//   const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
//   const double LA = fb->LA;
//   const double LB = fb->LB;
//   const double LC = fb->LC;
// // halt if assembly mode is ambiguous
//   const double assembly_mode_ambiguous_threshold = 1e-2;
//   const double r1px = current_xy[0] - LB*cos(current_ang[0]) + LC/2.0;
//   const double r1py = current_xy[1] - LB*sin(current_ang[0]);
//   const double r2px = current_xy[0] - LB*cos(current_ang[1]) - LC/2.0;
//   const double r2py = current_xy[1] - LB*sin(current_ang[1]);
//   const double r1pcorssr2p = r1px*r2py - r1py*r2px;
//   if (fabs(r1pcorssr2p/(LA*LA)) < assembly_mode_ambiguous_threshold) goto halt;
//   unsigned int current_mode = fb_setup_tell_mode(fb);
//   const double current_sdf_dst = sdf_lut_sample(current_xy[0], current_xy[1], current_mode);
//   if (current_sdf_dst < 0.08) {
//     double mode_handling_torques[2];
//     double frac = fabs(r1pcorssr2p/(LA*LA)) < 0.1 ? 0.0 : 0.5;
//     _mode_handling(mode_handling_torques, current_xy, current_ang, LA, LB, LC, current_mode, frac);
//     torque[0] = mode_handling_torques[0]*100.0;
//     torque[1] = mode_handling_torques[1]*100.0;
//     return;
//   }
//   if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
  
//   int target_in_same_mode = 0;

//   for (int i=0; i<4; i++) {
//     double dst = sdf_lut_sample(target[0], target[1], current_mode);
//     if (dst > 0.07) {
//       target_in_same_mode = 1;
//       break;
//     }
//   }
//   if (!target_in_same_mode) goto halt;
//   double new_target[2];
//   sdf_pick_direction(new_target, current_xy, target, 0.075, current_mode);
//   double J[4];
//   fb_setup_jacobian(J, fb);
//   double coriolis_accel[2];
//   fb_setup_coriolis_accel(coriolis_accel, fb);
//   const double err_xy[2] = {new_target[0]-current_xy[0], new_target[1]-current_xy[1]};
//   const double end_force[2] = {kp*err_xy[0] - kd*current_v[0], kp*err_xy[1] - kd*current_v[1]};
//   double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
//   torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
//   torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

//   const double torque_limit = 100.0;
//   const double torque_mag = sqrt(torque[0]*torque[0] + torque[1]*torque[1]);
//   if (torque_mag > torque_limit) {
//     torque[0] *= torque_limit/torque_mag;
//     torque[1] *= torque_limit/torque_mag;
//   }
//   return;
// halt:
//   const double kd_halt = 100.0;
//   torque[0] = -kd_halt*current_w[0];
//   torque[1] = -kd_halt*current_w[1];
// }

// void controller_test(double *torque, fb_setup *fb, const double *target, void *data)
// {
//   const double kp = 100.0;
//   const double kd = 50;
//   const double *current_xy = fb->state+6;
//   const double current_ang[2] = {fb->state[2], fb->state[13]};
//   const double *current_v = fb->state+(6+14);
//   const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
//   if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
//   unsigned int current_mode = fb_setup_tell_mode(fb);
//   int target_in_same_mode = 0;

//   for (int i=0; i<4; i++) {
//     double dst = sdf_lut_sample(target[0], target[1], current_mode);
//     if (dst > 0.075) {
//       target_in_same_mode = 1;
//       break;
//     }
//   }
//   if (!target_in_same_mode) goto halt;
//   double new_target[2];
//   sdf_pick_direction(new_target, current_xy, target, 0.075, current_mode);
//   double J[4];
//   fb_setup_jacobian(J, fb);
//   double coriolis_accel[2];
//   fb_setup_coriolis_accel(coriolis_accel, fb);
//   const double err_xy[2] = {new_target[0]-current_xy[0], new_target[1]-current_xy[1]};
//   const double end_force[2] = {kp*err_xy[0] - kd*current_v[0], kp*err_xy[1] - kd*current_v[1]};
//   double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
//   torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
//   torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

//   const double torque_limit = 100.0;
//   const double torque_mag = sqrt(torque[0]*torque[0] + torque[1]*torque[1]);
//   if (torque_mag > torque_limit) {
//     torque[0] *= torque_limit/torque_mag;
//     torque[1] *= torque_limit/torque_mag;
//   }
//   return;
// halt:
//   const double kd_halt = 100.0;
//   torque[0] = -kd_halt*current_w[0];
//   torque[1] = -kd_halt*current_w[1];
// }

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
  const double MB = 1.5; //0.1; //1.5;
  const double MLoad = 1.0;
  const double IA = 0.1;
  const double IB = 0.1; //1.0;
  fb_setup *fb = make_fb_setup(LA, LB, LC, MA, IA, MB, IB, MLoad);
  fb_setup_init_xy(fb, 0.0, 1.0, 2);
  // fb_setup_init_ang(fb, 0.0, 0.0, 1);

  // fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_naive, .controller_data=NULL, .target_xy={0.0, 1.5}, .running=1};
  // double display_data[8];
  // fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_test3, .controller_data=display_target_positions, .target_xy={0.0, 1.5}, .running=1};
  controller_memory controller_state = {.mode_crossing = 0};
  fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_test5, .controller_data=&controller_state, .target_xy={0.0, 1.5}, .running=0};
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
    // DrawCircleV((Vector2){origin.x+display_scale*display_target_positions[0], origin.y-display_scale*display_target_positions[1]}, 5.0, GREEN);
    // DrawCircleV((Vector2){origin.x+display_scale*display_target_positions[2], origin.y-display_scale*display_target_positions[3]}, 5.0, MAGENTA);
    DrawText(fb->assembly_mode==1 ? "ASSEMBLY MODE: +" : "ASSEMBLY MODE: -", 200, 60, 20, WHITE);

    // double load_pos[2] = {fb->state[6], fb->state[7]};

    DrawText((const char *[4]){"WORKING MODE: --", "WORKING MODE: +-", "WORKING MODE: -+", "WORKING MODE: ++"}[fb->working_mode], 200, 80, 20, WHITE);
    char simulation_time[30];
    snprintf(simulation_time, 30, "TIME: %.3f",simulation.fb->sim_time);
    DrawText(simulation_time, 200, 100, 20, WHITE);
    // DrawFPS(0, 0);
    EndDrawing();
  }
  pthread_cancel(thread_id);
  return 0;
}