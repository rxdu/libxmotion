/*
 * terminal.hpp
 *
 * Created on: Jul 11, 2022 13:06
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef TERMINAL_HPP
#define TERMINAL_HPP

#include <cstdint>
#include <atomic>
#include <memory>
#include <unordered_map>

#include "ncview/nc_window.hpp"

namespace robosw {
namespace swviz {
class Terminal {
 public:
  Terminal();
  ~Terminal();

  void AddWindow(std::shared_ptr<NcWindow> win);

  void Show(uint32_t fps = 60);

 protected:
  std::unordered_map<std::string, std::shared_ptr<NcWindow>> windows_;
  virtual void Update() {}

 private:
  std::atomic<bool> keep_running_{false};
};
}  // namespace swviz
}  // namespace robosw

#endif /* TERMINAL_HPP */
