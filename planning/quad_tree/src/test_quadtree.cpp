// standard libaray
#include <stdio.h>

// opencv
#include "opencv2/opencv.hpp"

// quad_tree
#include "qtree_builder.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat image_raw;
    image_raw = imread( argv[1], 1 );

    if ( !image_raw.data )
    {
        printf("No image data \n");
        return -1;
    }

    // convert image to gray
    Mat image_gray;
    cvtColor(image_raw, image_gray, COLOR_BGR2GRAY);

    // binarize image
    Mat image_bin;
    threshold(image_gray, image_bin, 200, 255, THRESH_BINARY);

    QTreeBuilder builder;

    // display final image
    //namedWindow("Original Image", WINDOW_AUTOSIZE );
    //imshow("Original Image", image_gray);

    Mat image_pad;
    builder.PadGrayscaleImage(image_bin, image_pad);

    namedWindow("Binarized Image", WINDOW_AUTOSIZE );
    imshow("Binarized Image", image_pad);

    waitKey(0);

    return 0;
}
