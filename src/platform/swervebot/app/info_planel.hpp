/*
 * @file info_planel.hpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_INFO_PLANEL_HPP
#define XMOTION_INFO_PLANEL_HPP

#include "imview/panel.hpp"

namespace xmotion {
class InfoPlanel : public quickviz::Panel {
 public:
  InfoPlanel();
  ~InfoPlanel() = default;

  void Draw() override;
};
}  // namespace xmotion

#endif  // XMOTION_INFO_PLANEL_HPP