/* 
 * figlabel.hpp
 * 
 * Created on: Jan 29, 2019 07:03
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef FIGLABEL_HPP
#define FIGLABEL_HPP

#include <string>

namespace librav
{
struct FigLabel
{
    FigLabel(const std::string &str = "", const double pos = 0) : str_(str), pos_(pos) {}

    std::string str_;
    double pos_;
};
} // namespace librav

#endif /* FIGLABEL_HPP */
