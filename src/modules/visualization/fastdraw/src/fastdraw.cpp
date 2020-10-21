/*
 * fastdraw.cpp
 *
 * Created on: Oct 21, 2020 22:12
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "fastdraw/fastdraw.hpp"

#include "raylib.h"

namespace ivnav {
FastCanvas::FastCanvas(int32_t width, int32_t height, std::string name)
    : width_(width), height_(height), name_(name) {
  InitWindow(width, height, name.c_str());
  SetTargetFPS(60);
}

FastCanvas::~FastCanvas() { CloseWindow(); }

void FastCanvas::Draw() {
  //   while (!WindowShouldClose())  // Detect window close button or ESC key
  //   {
  //     BeginDrawing();

  // ClearBackground(RAYWHITE);
  ClearCanvas(FCOLOR_RAYWHITE);

  DrawText("some basic shapes available on raylib", 20, 20, 20, DARKGRAY);

  DrawCircle(width_ / 4, 120, 35, DARKBLUE);

  DrawRectangle(width_ / 4 * 2 - 60, 100, 120, 60, RED);
  DrawRectangleLines(width_ / 4 * 2 - 40, 320, 80, 60,
                     ORANGE);  // NOTE: Uses QUADS internally, not lines
  DrawRectangleGradientH(width_ / 4 * 2 - 90, 170, 180, 130, MAROON, GOLD);

  DrawTriangle((Vector2){width_ / 4 * 3, 80},
               (Vector2){width_ / 4 * 3 - 60, 150},
               (Vector2){width_ / 4 * 3 + 60, 150}, VIOLET);

  DrawPoly((Vector2){width_ / 4 * 3, 320}, 6, 80, 0, BROWN);

  DrawCircleGradient(width_ / 4, 220, 60, GREEN, SKYBLUE);

  // NOTE: We draw all LINES based shapes together to optimize internal
  // drawing, this way, all LINES are rendered in a single draw pass
  DrawLine(18, 42, width_ - 18, 42, BLACK);
  DrawCircleLines(width_ / 4, 340, 80, DARKBLUE);
  DrawTriangleLines((Vector2){width_ / 4 * 3, 160},
                    (Vector2){width_ / 4 * 3 - 20, 230},
                    (Vector2){width_ / 4 * 3 + 20, 230}, DARKBLUE);
  //     EndDrawing();
  //   }
}

void FastCanvas::StartDrawing() { BeginDrawing(); }
void FastCanvas::FinishDrawing() { EndDrawing(); }
void FastCanvas::ClearCanvas(FColor bk_color) {
  Color color;
  color.r = bk_color.r;
  color.g = bk_color.g;
  color.b = bk_color.b;
  color.a = bk_color.a;
  ClearBackground(color);
}

void FastCanvas::Show() {
  while (!WindowShouldClose())  // Detect window close button or ESC key
  {
    BeginDrawing();
    Draw();
    EndDrawing();
  }
}
}  // namespace ivnav