/**
* @file nc_subwindow.hpp
* @date 7/11/22
* @brief a sub-window of stdscr
*
* @copyright Copyright (c) 2022 Ruixiang Du (rdu)
*/

#ifndef NC_SUBWINDOW_HPP
#define NC_SUBWINDOW_HPP

#include <ncurses.h>

#include <string>

#include "ncview/details/nc_element.hpp"
#include "ncview/nc_text.hpp"

namespace robosw {
namespace swviz {
class NcSubWindow : public NcElement {
 public:
  struct Specs {
    Specs() : name(""), pos({0, 0}),
              size({0, 0}), with_border(true) {};

    std::string name;
    NcVector2 pos;
    NcVector2 size;
    bool with_border;
  };

 public:
  NcSubWindow(const Specs &specs = {});
  virtual ~NcSubWindow();

  std::string GetName() const { return specs_.name; }

  void Update();

 protected:
  virtual void OnDraw() = 0;
  void OnResize(int rows, int cols, int y, int x) override;

  Specs specs_;
  WINDOW *window_;
  NcBox bounding_box_;
};
}  // namespace swviz
}  // namespace robosw

#endif /* NC_SUBWINDOW_HPP */
