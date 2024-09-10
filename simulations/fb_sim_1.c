#include <stdio.h>
#include <math.h>
// #include <unistd.h>
#include "usleep.h"
#include <pthread.h>
#include <assert.h>
#include "fb_util.h"
#include "raylib.h"

// gcc -lm -lopengl32 -pthread fb_util.c simple_linalg.c fb_sim_1.c usleep.c raylib.dll

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
  const double wm_grad1 = (to_mode&1 ? -1.0 : 1.0)*((xy[0]-LB*cos(ang[0])+LC/2.0)*cos(ang[0]) + (xy[1]-LB*sin(ang[0]))*sin(ang[0]))/LA;
  const double wm_grad2 = (to_mode&2 ? -1.0 : 1.0)*((xy[0]-LB*cos(ang[1])-LC/2.0)*cos(ang[1]) + (xy[1]-LB*sin(ang[1]))*sin(ang[1]))/LA;
  const double wm_mag = sqrt(wm_grad1*wm_grad1 + wm_grad2*wm_grad2);
  const double dr1r2s = LB*LB*((cos(ang[0])-cos(ang[1]))*(cos(ang[0])-cos(ang[1]))+(sin(ang[0])-sin(ang[1]))*(sin(ang[0])-sin(ang[1]))) - 2*LB*LC*(cos(ang[0])-cos(ang[1])) + LC*LC;
  const double am_grad1 = 2*LB*(LB*sin(ang[0]-ang[1])+LC*sin(ang[0]));
  const double am_grad2 = -2*LB*(LB*sin(ang[0]-ang[1])+LC*sin(ang[1]));
  const double am_mag = sqrt(am_grad1*am_grad1 + am_grad2*am_grad2);
  res[0] = frac*wm_grad1/wm_mag + (1.0-frac)*am_grad1/am_mag;
  res[1] = frac*wm_grad2/wm_mag + (1.0-frac)*am_grad2/am_mag;
}

void controller_test2(double *torque, fb_setup *fb, const double *target, void *data)
{
  const double kp = 100.0;
  const double kd = 50;
  const double *current_xy = fb->state+6;
  const double current_ang[2] = {fb->state[2], fb->state[13]};
  const double *current_v = fb->state+(6+14);
  const double current_w[2] = {fb->state[2+14], fb->state[13+14]};
  const double LA = fb->LA;
  const double LB = fb->LB;
  const double LC = fb->LC;
// halt if assembly mode is ambiguous
  const double assembly_mode_ambiguous_threshold = 1e-2;
  const double r1px = current_xy[0] - LB*cos(current_ang[0]) + LC/2.0;
  const double r1py = current_xy[1] - LB*sin(current_ang[0]);
  const double r2px = current_xy[0] - LB*cos(current_ang[1]) - LC/2.0;
  const double r2py = current_xy[1] - LB*sin(current_ang[1]);
  const double r1pcorssr2p = r1px*r2py - r1py*r2px;
  if (fabs(r1pcorssr2p/(LA*LA)) < assembly_mode_ambiguous_threshold) goto halt;
  unsigned int current_mode = fb_setup_tell_mode(fb);
  const double current_sdf_dst = sdf_lut_sample(current_xy[0], current_xy[1], current_mode);
  if (current_sdf_dst < 0.05) {
    double mode_handling_torques[2];
    _mode_handling(mode_handling_torques, current_xy, current_ang, LA, LB, LC, current_mode, 0.5);
    torque[0] = mode_handling_torques[0]*100.0;
    torque[1] = mode_handling_torques[1]*100.0;
    return;
  }
  if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
  
  int target_in_same_mode = 0;

  for (int i=0; i<4; i++) {
    double dst = sdf_lut_sample(target[0], target[1], current_mode);
    if (dst > 0.075) {
      target_in_same_mode = 1;
      break;
    }
  }
  if (!target_in_same_mode) goto halt;
  double new_target[2];
  sdf_pick_direction(new_target, current_xy, target, 0.075, current_mode);
  double J[4];
  fb_setup_jacobian(J, fb);
  double coriolis_accel[2];
  fb_setup_coriolis_accel(coriolis_accel, fb);
  const double err_xy[2] = {new_target[0]-current_xy[0], new_target[1]-current_xy[1]};
  const double end_force[2] = {kp*err_xy[0] - kd*current_v[0], kp*err_xy[1] - kd*current_v[1]};
  double IM = fb->IB + fb->MB*fb->LB*fb->LB/4.0;
  torque[0] = IM*coriolis_accel[0]+(J[0]*end_force[0]+J[2]*end_force[1]);
  torque[1] = IM*coriolis_accel[1]+(J[1]*end_force[0]+J[3]*end_force[1]);

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
  const double MB = 1.5;
  const double MLoad = 1.0;
  const double IA = 0.1;
  const double IB = 1.0;
  fb_setup *fb = make_fb_setup(LA, LB, LC, MA, IA, MB, IB, MLoad);
  fb_setup_init_xy(fb, 0.0, 1.0, 2);
  // fb_setup_init_ang(fb, 0.0, 0.0, 1);

  // fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_naive, .controller_data=NULL, .target_xy={0.0, 1.5}, .running=1};
  fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_test2, .controller_data=NULL, .target_xy={0.0, 1.5}, .running=1};
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
    DrawText(fb->assembly_mode==1 ? "ASSEMBLY MODE: +" : "ASSEMBLY MODE: -", 0, 20, 20, WHITE);
    DrawText((const char *[4]){"WORKING MODE: --", "WORKING MODE: +-", "WORKING MODE: -+", "WORKING MODE: ++"}[fb->working_mode], 0, 40, 20, WHITE);
    char simulation_time[30];
    snprintf(simulation_time, 30, "TIME: %.3f",simulation.fb->sim_time);
    DrawText(simulation_time, 0, 60, 20, WHITE);
    DrawFPS(0, 0);
    EndDrawing();
  }
  pthread_cancel(thread_id);
  return 0;
}