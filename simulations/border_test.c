#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "fb_util.h"
#include "raylib.h"

// gcc -lm -lopengl32 fb_util.c simple_linalg.c border_test.c raylib.dll -o 3.exe

#define WINDOW_W 1024
#define WINDOW_H 768

double wrap_angle(double x)
{
  while (x > 3.1415926535897932) x -= 2*3.1415926535897932;
  while (x < -3.1415926535897932) x += 2*3.1415926535897932;
  return x;
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

int modes_has_wm_border(unsigned int mode_from, unsigned int mode_to) {
  if ((mode_from &4) != (mode_to &4)) return 0;
  if ((mode_from&1) != (mode_to&1) && (mode_from&2) != (mode_to&2)) return 0;
  return 1;
}

double working_mode_border_position(double *res, const double *from, const double *to, unsigned int from_mode, unsigned int to_mode, double LA, double LB, double LC)
{
  const unsigned int border_id = (from_mode + to_mode)/2 + (((from_mode + to_mode)&7) == 5);
  assert(working_mode_border_type[border_id] > 0 && working_mode_border_type[border_id] <= 3);
  // printf("%u\n", border_id);
  const double x_shift = (border_id+1)&2 ? -LC/2.0 : LC/2.0;
  const double from_x = from[0] + x_shift;
  const double from_y = from[1];
  const double mag_from = sqrt(from_x*from_x + from_y*from_y);
  const double to_x = to[0] + x_shift;
  const double to_y = to[1];
  const double mag_to = sqrt(to_x*to_x + to_y*to_y);
  const double safe_distance = 0.04;
  const double safety_factor = 0.9;
  const double inner_radius = fabs(LA-LB) + safe_distance;
  const double outer_radius = LA+LB - safe_distance;
  double border_position_inner[2];
  double border_position_outer[2];
  if (working_mode_border_type[border_id] & 1) {
    const double ref_arg = atan2(from_y*fabs(mag_to-inner_radius)+to_y*fabs(mag_from-inner_radius), from_x*fabs(mag_to-inner_radius)+to_x*fabs(mag_from-inner_radius));
    const double ref_to_inner_center = wrap_angle(ref_arg - working_mode_border_center[2*border_id+0]);
    if (fabs(ref_to_inner_center) < safety_factor*working_mode_border_range[2*border_id+0]) {
      border_position_inner[0] = inner_radius*cos(ref_arg) - x_shift;
      border_position_inner[1] = inner_radius*sin(ref_arg);
    } else {
      const double arg = copysign(safety_factor*working_mode_border_range[2*border_id+0],ref_to_inner_center) + working_mode_border_center[2*border_id+0];
      border_position_inner[0] = inner_radius*cos(arg) - x_shift;
      border_position_inner[1] = inner_radius*sin(arg);
    }
  }
  if (working_mode_border_type[border_id] & 2) {
    const double ref_arg = atan2(from_y*fabs(mag_to-outer_radius)+to_y*fabs(mag_from-outer_radius), from_x*fabs(mag_to-outer_radius)+to_x*fabs(mag_from-outer_radius));
    const double ref_to_outer_center = wrap_angle(ref_arg - working_mode_border_center[2*border_id+1]);
    if (fabs(ref_to_outer_center) < safety_factor*working_mode_border_range[2*border_id+1]) {
      border_position_outer[0] = outer_radius*cos(ref_arg) - x_shift;
      border_position_outer[1] = outer_radius*sin(ref_arg);
    } else {
      const double arg = copysign(safety_factor*working_mode_border_range[2*border_id+1],ref_to_outer_center) + working_mode_border_center[2*border_id+1];
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
    if (side == 0) { // switch left
      switch (to) {
        case 1:
        case 2:
        case 5:
        case 6:
          res[0] = J[1]; res[1] = J[3]; return;
        case 0:
        case 3:
        case 4:
        case 7:
          res[0] = -J[1]; res[1] = -J[3]; return;
        default:
          assert(0);
      }
    } else { // switch right
      switch (to) {
        case 1:
        case 2:
        case 5:
        case 6:
          res[0] = -J[0]; res[1] = -J[2]; return;
        case 0:
        case 3:
        case 4:
        case 7:
          res[0] = J[0]; res[1] = J[2]; return;
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

void draw_fb_ws(Vector2 origin, double display_scale, double LA, double LB, double LC)
{
  DrawCircleLinesV((Vector2){origin.x - display_scale*LC/2.0, origin.y}, display_scale*fabs(LA-LB), BLUE);
  DrawCircleLinesV((Vector2){origin.x + display_scale*LC/2.0, origin.y}, display_scale*fabs(LA-LB), BLUE);
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
  fb_setup *fb = make_fb_setup(LA, LB, LC, 1.0, 1.0, 1.0, 1.0, 1.0);

  SetTraceLogLevel(LOG_ERROR);
  InitWindow(WINDOW_W, WINDOW_H, "fb_util test");
  SetTargetFPS(60);
  SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);
  unsigned int mode_current = 0;
  unsigned int mode_target = 0;
  const char *mode_current_display[8] = {
    "CURRENT MODE: 0", "CURRENT MODE: 1", "CURRENT MODE: 2", "CURRENT MODE: 3",
    "CURRENT MODE: 4", "CURRENT MODE: 5", "CURRENT MODE: 6", "CURRENT MODE: 7"
  };
  const char *mode_target_display[8] = {
    "TARGET MODE: 0", "TARGET MODE: 1", "TARGET MODE: 2", "TARGET MODE: 3",
    "TARGET MODE: 4", "TARGET MODE: 5", "TARGET MODE: 6", "TARGET MODE: 7"
  };
  const char *mode_switching_possible_dislay[2] = {
    "NO", "YES"
  };
  double current[2] = {0.0, 0.0};
  double target[2] = {0.0, 0.0};
  while (!WindowShouldClose()) {
    Vector2 origin = {WINDOW_W/2.0, WINDOW_H/2.0};
    double display_scale = 180.0;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      Vector2 mouse_pos = GetMousePosition();
      double new_x = (mouse_pos.x-origin.x)/display_scale;
      double new_y = -(mouse_pos.y-origin.y)/display_scale;
      target[0] = new_x;
      target[1] = new_y;
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      Vector2 mouse_pos = GetMousePosition();
      double new_x = (mouse_pos.x-origin.x)/display_scale;
      double new_y = -(mouse_pos.y-origin.y)/display_scale;
      current[0] = new_x;
      current[1] = new_y;
    }
    int key = GetKeyPressed();
    if (key != KEY_NULL) {
      switch (key) {
        case KEY_Q: mode_current = 0; break;
        case KEY_W: mode_current = 1; break;
        case KEY_E: mode_current = 2; break;
        case KEY_R: mode_current = 3; break;
        case KEY_T: mode_current = 4; break;
        case KEY_Y: mode_current = 5; break;
        case KEY_U: mode_current = 6; break;
        case KEY_I: mode_current = 7; break;
        case KEY_A: mode_target = 0; break;
        case KEY_S: mode_target = 1; break;
        case KEY_D: mode_target = 2; break;
        case KEY_F: mode_target = 3; break;
        case KEY_G: mode_target = 4; break;
        case KEY_H: mode_target = 5; break;
        case KEY_J: mode_target = 6; break;
        case KEY_K: mode_target = 7; break;
        default:
      }
    }
    const int mode_switching_possible = modes_has_wm_border(mode_current, mode_target);
    double border_position[2];
    BeginDrawing();
    ClearBackground((Color){0, 0, 0, 255});
    DrawCircleLinesV((Vector2){origin.x+display_scale*current[0], origin.y-display_scale*current[1]}, 5.0, RED);
    DrawCircleLinesV((Vector2){origin.x+display_scale*target[0], origin.y-display_scale*target[1]}, 5.0, VIOLET);
    draw_fb_ws(origin, display_scale, LA, LB, LC);
    DrawText(mode_current_display[mode_current], 0, 20, 20, WHITE);
    DrawText(mode_target_display[mode_target], 0, 40, 20, WHITE);
    if (mode_current != mode_target) {
      DrawText(mode_switching_possible_dislay[mode_switching_possible], 0, 60, 20, WHITE);
      if (mode_switching_possible) {
        double current_ang[2];
        fb_setup_ik(current_ang, fb, current[0], current[1], mode_current&3);
        double J[4];
        fb_setup_jacobian_at(J, fb, current, current_ang);
        double direction[2];
        int side;
        if ((mode_current&1) != (mode_target&1)) side = 0; else side = 1;
        wm_crossing_direction(direction, J, mode_current, mode_target, side, 0);
        DrawLineV((Vector2){origin.x+display_scale*current[0], origin.y-display_scale*current[1]},
          (Vector2){origin.x+(display_scale*current[0] + 10*J[0]), origin.y-(display_scale*current[1] + 10*J[2])}, GREEN);
        DrawLineV((Vector2){origin.x+display_scale*current[0], origin.y-display_scale*current[1]},
          (Vector2){origin.x+(display_scale*current[0] + 10*J[1]), origin.y-(display_scale*current[1] + 10*J[3])}, BROWN);
        DrawLineV((Vector2){origin.x+display_scale*current[0], origin.y-display_scale*current[1]},
          (Vector2){origin.x+(display_scale*current[0] + 20*direction[0]), origin.y-(display_scale*current[1] + 20*direction[1])}, VIOLET);
        working_mode_border_position(border_position, current, target, mode_current, mode_target, LA, LB, LC);
        char border_position_display[30];
      snprintf(border_position_display, 30, "x: %.3f, y: %.3f", border_position[0], border_position[1]);
      DrawText(border_position_display, 0, 80, 20, WHITE);
      DrawCircleV((Vector2){origin.x+display_scale*border_position[0], origin.y-display_scale*border_position[1]}, 3.0, GREEN);
      }
    } else {
      DrawText("IN SAME MODE", 0, 60, 20, WHITE);
    }
    DrawFPS(0, 0);
    EndDrawing();
  }
  return 0;
}