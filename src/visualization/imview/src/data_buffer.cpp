/*
 * data_buffer.cpp
 *
 * Created on: Mar 25, 2021 17:39
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/data_buffer.hpp"

#include <cmath>

namespace xmotion {
namespace swviz {
DataBuffer::DataBuffer(uint32_t size) : buffer_size_(size) {
  data_.reserve(size);
}

void DataBuffer::Resize(uint32_t size) {
  data_.reserve(size);
  buffer_size_ = size;
}

std::size_t DataBuffer::GetSize() const { return data_.size(); }

std::size_t DataBuffer::GetOffset() const { return offset_; }

void DataBuffer::Erase() {
  if (data_.size() > 0) {
    data_.shrink(0);
    offset_ = 0;
  }
}

void DataBuffer::AddPoint(float x, float y) {
  if (data_.size() < buffer_size_)
    data_.push_back(ImVec2(x, y));
  else {
    data_[offset_] = ImVec2(x, y);
    offset_ = (offset_ + 1) % buffer_size_;
  }
}

ImVec2 &DataBuffer::operator[](std::size_t index) {
  assert(index < data_.size());
  return data_[index];
}
}  // namespace swviz
}  // namespace xmotion