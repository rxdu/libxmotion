/* 
 * popup.hpp
 *
 * Created on 4/5/22 11:08 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_POPUP_HPP
#define ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_POPUP_HPP

#include <string>
#include <cstdint>

namespace xmotion {
namespace swviz {
bool ShowPopupNotification(std::string msg, std::string title,
                           float width = 300, float height = 150);
}
}

#endif //ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_POPUP_HPP
