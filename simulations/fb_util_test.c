#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include "fb_util.h"
#include "raylib.h"

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
    useconds_t usecs = (unsigned int)(((fb_sim *)p)->dt*1e6);
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

void controller_naive(double *torque, fb_setup *fb, const double *target, void *data)
{
  const double kp = 100.0;
  const double kd = 50.0;
  if (!fb_setup_is_xy_reachable(fb, target[0], target[1])) goto halt;
  double target_ang[2];
  double ang_candidates[8];
  // double ang_distances[4];
  double ang_dst_min = 999.0;
  int ang_candidate_min = -1;
  // fb_setup_ik(target_ang, fb, target[0], target[1], fb->working_mode);
  for (int i=0; i<4; i++) {
    fb_setup_ik(ang_candidates+2*i, fb, target[0], target[1], i);
    double dst_th1 = wrap_angle((ang_candidates+2*i)[0] - fb->state[2]);
    double dst_th2 = wrap_angle((ang_candidates+2*i)[1] - fb->state[13]);
    double ang_distance = dst_th1*dst_th1 + dst_th2*dst_th2;
    if (fb_setup_sgl2_condition_at(fb, ang_candidates[2*i], ang_candidates[2*i+1])>0.3 && fb_setup_tell_assembly_mode_at(fb, target, ang_candidates+2*i)==fb->assembly_mode && ang_distance < ang_dst_min) {
      ang_dst_min = ang_distance;
      ang_candidate_min = i;
    }
  }
  if (ang_candidate_min == -1) goto halt;
  target_ang[0] = ang_candidates[2*ang_candidate_min+0];
  target_ang[1] = ang_candidates[2*ang_candidate_min+1];
  double err_th1 = atan2(sin(target_ang[0]-fb->state[2]), cos(target_ang[0]-fb->state[2]));
  double err_th2 = atan2(sin(target_ang[1]-fb->state[13]), cos(target_ang[1]-fb->state[13]));

  double penalty[2];
  fb_setup_sgl2_condition_penalty(penalty, fb);
  double w[2] = {fb->state[2+14], fb->state[13+14]};
  double alignment = penalty[0]*w[0] + penalty[1]*w[1];
  if (alignment > 0.0) {
    torque[0] = kp*err_th1 - kd*fb->state[2+14];
    torque[1] = kp*err_th2 - kd*fb->state[13+14];
  } else {
    torque[0] = kp*err_th1 -kd*(w[0]+0.1*alignment*penalty[0]);
    torque[1] = kp*err_th2 -kd*(w[1]+0.1*alignment*penalty[1]);
  }
  // torque[0] = kp*err_th1 - kd*fb->state[2+14];
  // torque[1] = kp*err_th2 - kd*fb->state[13+14];
  return;
halt:
  torque[0] = -kd*fb->state[2+14];
  torque[1] = -kd*fb->state[13+14];
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
  const double MLoad = 2.0;
  const double IA = 0.2;
  const double IB = 1.0;
  fb_setup *fb = make_fb_setup(LA, LB, LC, MA, IA, MB, IB, MLoad);
  fb_setup_init_xy(fb, 0.0, 1.0, 2);
  // fb_setup_init_ang(fb, 0.0, 0.0, 1);

  fb_sim simulation = {.dt=0.001, .fb=fb, .controller=controller_naive, .controller_data=NULL, .target_xy={0.0, 1.5}, .running=1};
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, fb_sim_runner, &simulation);

  SetTraceLogLevel(LOG_ERROR);
  InitWindow(WINDOW_W, WINDOW_H, "fb_util test");
  SetTargetFPS(60);
  SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);
  while (!WindowShouldClose()) {
    ClearBackground((Color){0, 0, 0, 255});
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
    BeginDrawing();
    DrawCircleLinesV((Vector2){origin.x+display_scale*simulation.target_xy[0], origin.y-display_scale*simulation.target_xy[1]},
      10.0, GREEN);
    draw_fb_state(fb, origin, display_scale);
    DrawText(fb->assembly_mode==1 ? "ASSEMBLY MODE: +" : "ASSEMBLY MODE: -", 0, 20, 20, WHITE);
    DrawText((const char *[4]){"WORKING MODE: --", "WORKING MODE: +-", "WORKING MODE: -+", "WORKING MODE: ++"}[fb->working_mode], 0, 40, 20, WHITE);
    DrawFPS(0, 0);
    EndDrawing();
  }
  pthread_cancel(thread_id);
  return 0;
}