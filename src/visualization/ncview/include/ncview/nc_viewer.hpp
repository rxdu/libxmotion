/**
* @file nc_viewer.hpp
* @date 7/11/22
* @brief a wrapper of stdscr, as main window
*
* @copyright Copyright (c) 2022 Ruixiang Du (rdu)
*/

#ifndef NCVIEWER_HPP
#define NCVIEWER_HPP

#include <cstdint>
#include <atomic>
#include <memory>
#include <unordered_map>

#include "ncview/nc_subwindow.hpp"

namespace robosw {
namespace swviz {
class NcViewer {
 public:
  NcViewer(const std::string &title = "NcViewer");
  ~NcViewer();

  void AddSubWindow(std::shared_ptr<NcSubWindow> win);
  void Show(uint32_t fps = 30);

 private:
  void Init();
  void Deinit();

  std::string title_;
  bool resize_triggered_;
  std::atomic<bool> keep_running_{false};
  std::unordered_map<std::string, std::shared_ptr<NcSubWindow>> sub_wins_;
  int term_size_x_ = 0;
  int term_size_y_ = 0;
};
}  // namespace swviz
}  // namespace robosw

#endif /* NCVIEWER_HPP */
