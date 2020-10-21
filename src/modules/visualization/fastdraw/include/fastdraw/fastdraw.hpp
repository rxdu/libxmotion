/*
 * fastdraw.hpp
 *
 * Created on: Oct 21, 2020 22:11
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef FASTDRAW_HPP
#define FASTDRAW_HPP

#include "details/fasttypes.hpp"

namespace ivnav {
class FastCanvas {
 public:
  FastCanvas(int32_t width, int32_t height, std::string name = "FastDraw");
  ~FastCanvas();

  virtual void Draw();

  void Show();

  void StartDrawing();
  void FinishDrawing();
  void ClearCanvas(FColor bk_color = FCOLOR_RAYWHITE);

 private:
  int32_t width_;
  int32_t height_;
  std::string name_;
};
}  // namespace ivnav

#endif /* FASTDRAW_HPP */
