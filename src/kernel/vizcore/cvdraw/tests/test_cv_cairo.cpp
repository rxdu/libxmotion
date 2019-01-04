#include <iostream>
#include <vector>
#include <stdio.h>
#include "opencv2/opencv.hpp"

// #include "cairo/cairo-win32.h"
#include "cairo/cairo-pdf.h"
#include "cairo/cairo-ps.h"
#include "cairo/cairo-svg.h"
using namespace std;
using namespace cv;

// source: https://stackoverflow.com/questions/19948319/how-to-convert-cairo-image-surface-to-opencv-mat-in-c

//-----------------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------------
void MatToCairo(Mat &MC3, cairo_surface_t *surface)
{
    Mat MC4 = Mat(cairo_image_surface_get_width(surface), cairo_image_surface_get_height(surface), CV_8UC4, cairo_image_surface_get_data(surface), cairo_image_surface_get_stride(surface));
    vector<Mat> Imgs1;
    vector<Mat> Imgs2;
    cv::split(MC4, Imgs1);
    cv::split(MC3, Imgs2);
    for (int i = 0; i < 3; i++)
    {
        Imgs1[i] = Imgs2[i];
    }
    // Alpha - прозрачность
    Imgs1[3] = 255;
    cv::merge(Imgs1, MC4);
}
//-----------------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------------
void CairoToMat(cairo_surface_t *surface, Mat &MC3)
{
    Mat MC4 = Mat(cairo_image_surface_get_width(surface), cairo_image_surface_get_height(surface), CV_8UC4, cairo_image_surface_get_data(surface), cairo_image_surface_get_stride(surface));
    vector<Mat> Imgs1;
    vector<Mat> Imgs2;
    cv::split(MC4, Imgs1);
    cv::split(MC3, Imgs2);
    for (int i = 0; i < 3; i++)
    {
        Imgs2[i] = Imgs1[i];
    }
    cv::merge(Imgs2, MC3);
}
//-----------------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    int m_spcount;
    double m_compactness(0);
    int width;
    int height;
    Mat Img = imread("/home/rdu/Pictures/lattice.png");
    namedWindow("Image");
    width = Img.cols;
    height = Img.rows;

    cairo_surface_t *surface;
    cairo_t *cr;
    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
    cr = cairo_create(surface);

    MatToCairo(Img, surface);

    // cairo_select_font_face(cr, "serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    // cairo_set_font_size(cr, 32.0);
    // cairo_set_source_rgb(cr, 0.0, 0.0, 1.0);
    // cairo_move_to(cr, 10.0, 50.0);
    // cairo_show_text(cr, "Hello, world");

    // cairo_surface_write_to_png(surface, "hello.png");

    // CairoToMat(surface, Img);

    imshow("Image", Img);
    waitKey(0);
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    destroyAllWindows();

    return 0;
}