/* 
 * image_io.cpp
 * 
 * Created on: Mar 23, 2018 17:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "image/image_io.hpp"

#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "image/stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "image/stb_image_write.h"

using namespace librav;

MonoImageMatrix ImageIO::ReadImage(std::string file_name)
{
    MonoImageMatrix image;

    int x, y, n;
    unsigned char *data = stbi_load(file_name.c_str(), &x, &y, &n, 0);

    if (data == NULL)
        return image;

    image.w = x;
    image.h = y;

    std::cout << "read image size (w/x,h/y,d): " << x << " , " << y << " , " << n << std::endl;

    image.data.resize(y, x);

    if (n <= 2)
    {
        for (int32_t j = 0; j < y; ++j)
            for (int32_t i = 0; i < x; ++i)
                image.data(j, i) = data[j * (x * n) + i * n];
    }
    else
    {
        // reference: https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor
        for (int32_t j = 0; j < y; ++j)
            for (int32_t i = 0; i < x; ++i)
                image.data(j, i) = 0.299 * data[j * (x * n) + i * n] + 0.587 * data[j * (x * n) + i * n + 1] + 0.114 * data[j * (x * n) + i * n + 2];
    }

    stbi_image_free(data);

    return image;
}

bool ImageIO::SaveToImage(const MonoImageMatrix &matrix, std::string file_name)
{
    bool save_to_png = true;

    bool found_jpg_suffix_in_name = (file_name.find(".jpg") != std::string::npos) || (file_name.find(".JPG") != std::string::npos);
    bool found_png_suffix_in_name = (file_name.find(".png") != std::string::npos) || (file_name.find(".PNG") != std::string::npos);

    // save to png by default, if user doesn't specify suffix
    if (found_jpg_suffix_in_name)
        save_to_png = false;
    else if(!found_png_suffix_in_name)
        file_name += ".png";

    int64_t size = matrix.w * matrix.h * matrix.d;
    int8_t *data = (int8_t *)malloc(size * sizeof(int8_t));
    for (int32_t j = 0; j < matrix.h; ++j)
        for (int32_t i = 0; i < matrix.w; ++i)
            data[j * matrix.w + i] = matrix.data(j, i);
    bool result;
    if (save_to_png)
        result = stbi_write_png(file_name.c_str(), matrix.w, matrix.h, matrix.d, data, matrix.w);
    else
        result = stbi_write_jpg(file_name.c_str(), matrix.w, matrix.h, matrix.d, data, 100);
    free(data);

    return result;
}