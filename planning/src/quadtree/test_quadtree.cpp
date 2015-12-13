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
    image_raw = imread( argv[1], IMREAD_GRAYSCALE );

    if ( !image_raw.data )
    {
        printf("No image data \n");
        return -1;
    }

    // convert image to gray
//    Mat image_gray;
//    cvtColor(image_raw, image_gray, COLOR_BGR2GRAY);

    // binarize image
//    Mat image_bin;
//    threshold(image_raw, image_bin, 200, 255, THRESH_BINARY);

//    QTreeBuilder builder;

    // display final image
    //namedWindow("Original Image", WINDOW_AUTOSIZE );
    //imshow("Original Image", image_gray);

//    Mat image_pad;
//    builder.PadGrayscaleImage(image_bin, image_pad);

    // example to use quadtree builder
    QTreeBuilder builder;
//    builder.BuildQuadTree(image_raw, 1);
    builder.BuildExtQuadTree(image_raw, 6);

    Mat image_tree;
    builder.VisualizeExtQuadTree(image_tree, TreeVisType::ALL_SPACE);

//    imwrite( "quadtree_freenodes.jpg", image_tree );

    namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
    imshow("Processed Image", image_tree);

    waitKey(0);

    return 0;
}
