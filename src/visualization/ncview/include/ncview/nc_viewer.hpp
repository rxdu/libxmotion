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
#include <vector>
#include <unordered_map>

#include "ncview/nc_subwindow.hpp"

namespace xmotion {
namespace swviz {
class NcViewer {
  enum class TitleOption {
    kNone = 0,
    kWithTitleOnly,
    kWithBorderOnly,
    kWithBorderAndTitle
  };

 public:
  NcViewer(const std::string &title = "", bool has_border = true);
  ~NcViewer();

  NcRegion GetDisplayRegion() const { return disp_region_; }

  // add a sub-window with user defined size and position
  //  void AddSubWindow(std::shared_ptr<NcSubWindow> win);
  void AddElement(std::shared_ptr<NcElement> element);

  void Show(uint32_t fps = 30);

 private:
  void Init();
  void Deinit();

  void CalcDisplayRegion();

  std::string title_;
  bool has_border_;
  TitleOption title_option_ = TitleOption::kWithBorderOnly;

  bool resize_triggered_;
  std::atomic<bool> keep_running_{false};
  //  std::unordered_map<std::string, std::shared_ptr<NcSubWindow>> sub_wins_;
  std::vector<std::shared_ptr<NcElement>> elements_;
  int term_size_x_ = 0;
  int term_size_y_ = 0;

  NcRegion disp_region_;
};
}  // namespace swviz
}  // namespace xmotion

#endif /* NCVIEWER_HPP */
