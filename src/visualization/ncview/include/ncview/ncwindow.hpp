/*
 * ncwindow.hpp
 *
 * Created on: Jul 11, 2022 14:56
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef NC_WINDOW_HPP
#define NC_WINDOW_HPP

#include <ncurses.h>

#include <string>

namespace robosw {
namespace swviz {
class NcViewer;

class NcWindow {
 public:
  NcWindow(std::string name, NcViewer* parent);
  ~NcWindow();

  std::string GetName() const { return name_; }

  virtual void Draw() = 0;

  void Clear();
  void Refresh();

 protected:
  std::string name_;
  const NcViewer* parent_;
  WINDOW* window_;

  int term_size_x_;
  int term_size_y_;
};
}  // namespace swviz
}  // namespace robosw

#endif /* NC_WINDOW_HPP */
