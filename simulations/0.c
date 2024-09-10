#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "raylib.h"

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
    DrawCircleV((Vector2){cur[0]*150.0 + 400.0, -cur[1]*150.0 + 300.0}, 3.0, GRAY);
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
    DrawCircleV((Vector2){cur[0]*150.0 + 400.0, -cur[1]*150.0 + 300.0}, 3.0, GRAY);
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

static void draw_texture_at(Texture t, Vector2 screen_pos, float scale, Color tint)
{
  DrawTexturePro(t, (Rectangle){.x=0, .y=0, .width=t.width, .height=-t.height},
    (Rectangle){.x=screen_pos.x, .y=screen_pos.y, .width=scale*t.width, .height=scale*t.height},
    (Vector2){scale*t.width/2.0, scale*t.height/2.0}, 0, tint);
}

#define WINDOW_W 800
#define WINDOW_H 600

int main(void)
{
  SetTraceLogLevel(LOG_ERROR);
  InitWindow(800, 600, "0");
  SetTargetFPS(60);
  SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);
  Vector2 cur_pos = {0.0f, 0.0f};
  Vector2 target_pos = {0.0f, 1.0f};
  Vector2 display_origin = {WINDOW_W/2.0f, WINDOW_H/2.0f};
  while(!WindowShouldClose()) {
    const float display_scale = 150;
    Vector2 mouse_pos = GetMousePosition();
    Vector2 mouse_coord = {(mouse_pos.x-display_origin.x)/display_scale, (display_origin.y-mouse_pos.y)/display_scale};
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      cur_pos = mouse_coord;
    } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      target_pos = mouse_coord;
    }
    double cur[2] = {cur_pos.x, cur_pos.y};
    double target[2] = {target_pos.x, target_pos.y};
    BeginDrawing();
    ClearBackground((Color){0,0,0,255});
    DrawCircleLinesV((Vector2){cur_pos.x*display_scale + display_origin.x, -cur_pos.y*display_scale + display_origin.y}, 5.0, RED);
    DrawCircleLinesV((Vector2){target_pos.x*display_scale + display_origin.x, -target_pos.y*display_scale + display_origin.y}, 5.0, VIOLET);
    EndDrawing();
  }
  return 0;
}