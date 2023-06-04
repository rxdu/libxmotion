
/**
 * @file nc_types.hpp
 * @date 1/29/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TYPES_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TYPES_HPP_

namespace xmotion {
namespace swviz {
struct NcVector2 {
  int x = 0;
  int y = 0;
};

struct NcRange {
  int min = 0;
  int max = 0;
};

struct NcRegion {
  NcVector2 pos;
  NcVector2 size;
};

struct NcBox {
  NcRange x;
  NcRange y;
};
}  // namespace swviz
}  // namespace xmotion

#endif  // ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_TYPES_HPP_
