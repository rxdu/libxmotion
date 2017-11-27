/* 
 * image_plot.h
 * 
 * Created on: Nov 27, 2017 16:47
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef IMAGE_PLOT_H
#define IMAGE_PLOT_H

#include "CImg/CImg.h"

namespace librav
{
namespace ImagePlot
{
    // template<typename T>
    void ShowImage(cimg_library::CImg<unsigned char>& img);
}
}

#endif /* IMAGE_PLOT_H */
