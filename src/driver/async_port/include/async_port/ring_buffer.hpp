/*
 * ring_buffer.hpp
 *
 * Created on: Dec 08, 2019 22:22
 * Updated on: Oct 2024
 * Description:
 *
 * Requirements:
 *  1. Size of buffer must be a power of 2
 *  2. Maximum buffer size is 2^(number_of_bits_of(size_t))-1
 *
 * Implementation details:
 *
 * - Initial state (empty)
 * [0][1][2][3]...[N]
 *  ^
 * R/W
 *
 * - Add one element
 * [D][1][2][3]...[N]
 *  ^  ^
 *  R  W
 *
 * - Buffer gets full when last element X is inserted and W moves to N.
 * The last position N will not be used for data storage because otherwise
 * W will move to the same position with R after the write and we cannot
 * differentiate between empty and full.
 * [D][D][D][D]...[X][N]
 *  ^                 ^
 *  R                 W
 *
 * - Buffer data pointed by R is overwritten (? not readable) when a new element
 * Y is inserted after the buffer is full. In other words, the R index is pushed
 * forward by one position after the buffer is full.
 * [?][D][D][D]...[X][Y]
 *  ^  ^
 *  W  R
 *
 * To differentiate between empty and full, one slot is always left empty before
 *  the read index. This is why the buffer size is N-1.
 *
 * Reference:
 *  [1]
 * https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
 *  [2]
 * https://stackoverflow.com/questions/10527581/why-must-a-ring-buffer-size-be-a-power-of-2
 *  [3]
 * https://stackoverflow.com/questions/9718116/improving-c-circular-buffer-efficiency
 *  [4] https://www.snellman.net/blog/archive/2016-12-13-ring-buffers/
 *  [5]
 * http://www.trytoprogram.com/c-examples/c-program-to-test-if-a-number-is-a-power-of-2/
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <cstdint>
#include <cstddef>
#include <cassert>

#include <mutex>
#include <array>
#include <vector>
#include <iostream>

namespace xmotion {
template <typename T = uint8_t, std::size_t N = 1024>
class RingBuffer {
 public:
  // Init and reset of buffer
  RingBuffer(bool enable_overwrite = true)
      : enable_overwrite_(enable_overwrite) {
    // assert size is a power of 2
    static_assert(
        (N != 0) && ((N & (N - 1)) == 0),
        "Size of ring buffer has to be 2^n, where n is a positive integer");

    size_mask_ = N - 1;
    write_index_ = 0;
    read_index_ = 0;
  }

  void Reset() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    write_index_ = 0;
    read_index_ = 0;
  }

  // Buffer size information
  bool IsEmpty() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return (read_index_ == write_index_);
  }

  bool IsFull() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return ((write_index_ + 1) & size_mask_) == (read_index_ & size_mask_);
  }

  std::size_t GetFreeSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return N - 1 - ((write_index_ - read_index_) & size_mask_);
  }

  std::size_t GetOccupiedSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return (write_index_ - read_index_) & size_mask_;
  };

  std::array<T, N> GetBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_;
  }

  // Read/Write functions
  // burst read/write functions
  std::size_t Read(std::vector<T>& data, std::size_t btr) {
    assert(data.size() >= btr);
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    // duplicated the read logic to avoid locking multiple times
    for (std::size_t i = 0; i < btr; ++i) {
      if (read_index_ == write_index_) return i;
      data[i] = buffer_[read_index_++ & size_mask_];
    }
    return btr;
  }

  std::size_t Peek(std::vector<T>& data, std::size_t btp) const {
    assert(data.size() >= btp);
    std::size_t count = 0;
    for (std::size_t i = 0; i < btp; ++i) {
      if (PeekAt(data[i], i) == 0) return i;
      count++;
    }
    return count;
  }

  std::size_t Write(const std::vector<T>& new_data, std::size_t btw) {
    assert(new_data.size() >= btw);
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    // duplicated the write logic to avoid locking multiple times
    for (std::size_t i = 0; i < btw; ++i) {
      if (((write_index_ + 1) & size_mask_) == (read_index_ & size_mask_)) {
        // return 0 if buffer is full and overwrite is disabled
        if (!enable_overwrite_) return i;
        // otherwise, advance the read_index to overwrite old data
        read_index_ = (read_index_ + 1) & size_mask_;
      }
      buffer_[(write_index_++) & size_mask_] = new_data[i];
    }
    return btw;
  }

  // single read/write functions
  std::size_t Read(T& data) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (read_index_ == write_index_) return 0;  // Buffer is empty
    data = buffer_[read_index_++ & size_mask_];
    return 1;
  }

  std::size_t PeekAt(T& data, size_t n) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    // return 0 if requested data is beyond the available range
    if (n >= (write_index_ - read_index_) & size_mask_) return 0;
    data = buffer_[(read_index_ + n) & size_mask_];
    return 1;
  }

  std::size_t Write(const T& new_data) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    // equivalent to if(IsFull()), avoid locking twice
    if (((write_index_ + 1) & size_mask_) == (read_index_ & size_mask_)) {
      // return 0 if buffer is full and overwrite is disabled
      if (!enable_overwrite_) return 0;
      // otherwise, advance the read_index to overwrite old data
      read_index_ = (read_index_ + 1) & size_mask_;
    }

    buffer_[(write_index_++) & size_mask_] = new_data;
    return 1;
  }

  void PrintBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::cout << "read index: " << read_index_
              << " , write index: " << write_index_ << std::endl;
    std::cout << "buffer content: " << std::endl;
    for (std::size_t i = 0; i < N; ++i)
      std::cout << "[" << i << "]"
                << " " << static_cast<int>(buffer_[i]) << std::endl;
  }

 private:
  std::array<T, N> buffer_;        // buffer memory to store data
  std::size_t size_mask_;          // for internal indexing management
  std::size_t write_index_ = 0;    // new data to be written
  std::size_t read_index_ = 0;     // the earliest written data to be read from
  bool enable_overwrite_ = false;  // enable buffer overwrite when full

  mutable std::mutex buffer_mutex_;
};
}  // namespace xmotion

#endif /* RING_BUFFER_HPP */
