#include "fastplot/image_plot.h"

using namespace librav;
using namespace cimg_library;

int main()
{
    CImg<unsigned char> img(300, 200, 1, 3);
    img.fill(32);
    //img.noise(128);

    const unsigned char white[] ={255,255,255};
    img.draw_text(80, 80, "HelloWorld", white, 0, 32);

    unsigned short color[5] ={0,8,16,24,32};
    img.draw_line(0,20,300,20, color);
    img.display();

    // ImagePlot::ShowImage(img);

    return 0;
}