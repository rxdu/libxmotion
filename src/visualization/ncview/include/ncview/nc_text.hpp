
/**
 * @file nc_text.hpp
 * @date 1/29/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TEXT_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TEXT_HPP_

#include <string>

#include "ncview/details/nc_element.hpp"

namespace robosw {
namespace swviz {
class NcText : public NcElement {
 public:
  NcText(const std::string &text);
  ~NcText() = default;

  // common interface
  void OnDraw() {}

 private:
  std::string text_;
};
}  // namespace swviz
}  // namespace robosw

#endif  // ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TEXT_HPP_
