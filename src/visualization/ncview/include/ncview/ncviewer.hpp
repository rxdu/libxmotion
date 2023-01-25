/*
 * ncviewer.hpp
 *
 * Created on: Jul 11, 2022 13:06
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef NCVIEWER_HPP
#define NCVIEWER_HPP

#include <cstdint>
#include <atomic>
#include <memory>
#include <unordered_map>

#include "ncview/ncwindow.hpp"

namespace robosw {
namespace swviz {
class NcViewer {
 public:
  NcViewer();
  ~NcViewer();

  void AddWindow(std::shared_ptr<NcWindow> win);

  void Show(uint32_t fps = 30);

 protected:
  std::unordered_map<std::string, std::shared_ptr<NcWindow>> windows_;
  virtual void Update() {}

 private:
  bool resize_triggered_;
  std::atomic<bool> keep_running_{false};

  void Init();
  void Deinit();
};
}  // namespace swviz
}  // namespace robosw

#endif /* NCVIEWER_HPP */
