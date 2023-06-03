
/**
 * @file nc_text.hpp
 * @date 1/29/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TEXT_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TEXT_HPP_

#include <ncurses.h>

#include <string>

namespace xmotion {
namespace swviz {
class NcText {
 public:
  enum Color : short {
    kDefault = -1,
    kBlack = 0,
    kRed = 1,
    kGreen = 2,
    kYellow = 3,
    kBlue = 4,
    kMagenta = 5,
    kCyan = 6,
    kWhite = 7,
    kLastColor = 8
  };

  static constexpr int ATTR_NORMAL = A_NORMAL;
  static constexpr int ATTR_STANDOUT = A_STANDOUT;
  static constexpr int ATTR_UNDERLINE = A_UNDERLINE;
  static constexpr int ATTR_REVERSE = A_REVERSE;
  static constexpr int ATTR_BLINK = A_BLINK;
  static constexpr int ATTR_DIM = A_DIM;
  static constexpr int ATTR_BOLD = A_BOLD;
  static constexpr int ATTR_PROTECT = A_PROTECT;
  static constexpr int ATTR_INVIS = A_INVIS;
  static constexpr int ATTR_ALTCHARSET = A_ALTCHARSET;

 public:
  static void InitColor(Color bg = Color::kDefault);

  // TODO (rdu) add support for changing background color
  static void SetBackgroundColor(Color bg);
  static void ResetBackgroundColor();

  static void SetAttribute(WINDOW *win, int attr);
  static void ResetAttribute(WINDOW *win);

  template <typename... Args>
  static void Printw(WINDOW *win, int y, int x, NcText::Color color,
                     const char *fmt, Args... args) {
    wattron(win, COLOR_PAIR(color));
    mvwprintw(win, y, x, fmt, args...);
    wattroff(win, COLOR_PAIR(color));
  }

 private:
  static int active_attributes;
};  // namespace NcText
}  // namespace swviz
}  // namespace xmotion

#endif  // ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TEXT_HPP_
