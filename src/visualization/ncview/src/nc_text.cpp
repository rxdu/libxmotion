/**
 * @file nc_text.cpp
 * @date 1/29/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "ncview/nc_text.hpp"

#include <iostream>

namespace xmotion {
namespace swviz {
int NcText::active_attributes = 0;

void NcText::InitColor(Color bg) {
  if (has_colors()) {
    use_default_colors();
    start_color();
    for (int i = NcText::Color::kDefault; i < NcText::Color::kLastColor; ++i) {
      init_pair(i, i, bg);
    }
  }
}

void NcText::SetBackgroundColor(NcText::Color bg) {
  for (int i = NcText::Color::kDefault; i < NcText::Color::kLastColor; ++i) {
    init_pair(i, i, bg);
  }
}

void NcText::ResetBackgroundColor() {
  for (int i = NcText::Color::kDefault; i < NcText::Color::kLastColor; ++i) {
    init_pair(i, i, -1);
  }
}

void NcText::SetAttribute(WINDOW *win, int attr) {
  active_attributes = attr;
  wattron(win, active_attributes);
}

void NcText::ResetAttribute(WINDOW *win) { wattroff(win, active_attributes); }
}  // namespace swviz
}  // namespace xmotion