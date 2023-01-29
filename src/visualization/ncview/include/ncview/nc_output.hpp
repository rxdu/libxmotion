/**
 * @file ncoutput.hpp
 * @date 25-01-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef NCVIEW_NCOUTPUT_HPP
#define NCVIEW_NCOUTPUT_HPP

#include <string>

namespace robosw {
namespace swviz {
namespace NcOutput {
struct NcColor {};

struct NcText {
  std::string text;
};
};  // namespace NcOutput
}  // namespace swviz
}  // namespace robosw

#endif /* NCVIEW_NCOUTPUT_HPP */
