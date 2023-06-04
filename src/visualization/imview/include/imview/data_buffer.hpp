/*
 * data_buffer.hpp
 *
 * Created on: Mar 25, 2021 14:29
 * Description: adapted from class ScrollingBuffer from implot_demo.cpp
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef PLOT_BUFFER_HPP
#define PLOT_BUFFER_HPP

#include <cstdint>
#include <mutex>
#include <vector>

#include "imgui.h"

namespace xmotion {
namespace swviz {
class DataBuffer {
 public:
  DataBuffer(uint32_t size = 2048);

  void Resize(uint32_t size);
  std::size_t GetSize() const;
  std::size_t GetOffset() const;

  void Erase();
  void AddPoint(float x, float y);
  ImVector<ImVec2>& GetData() { return data_; }
  ImVec2& operator[](std::size_t index);

 private:
  ImVector<ImVec2> data_;
  uint32_t buffer_size_ = 0;
  uint32_t offset_ = 0;
};
}  // namespace swviz
}  // namespace xmotion

#endif /* PLOT_BUFFER_HPP */
