/* 
 * image_plot.cpp
 * 
 * Created on: Nov 27, 2017 16:47
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "fastplot/image_plot.h"

using namespace librav::ImagePlot;
using namespace cimg_library;

//template<typename T>
void ShowImage(CImg<unsigned char>& img)
{
    img.display();
}