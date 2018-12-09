/* 
 * viewer_utils.cpp
 * 
 * Created on: Nov 20, 2018 22:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightview/viewer_utils.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <GL/gl3w.h>

using namespace librav;

// Source: https://gist.github.com/saccadic/2a40b09e3fc4623cb14c02cac3ddbe6c
// Reference:
//  [1] https://stackoverflow.com/questions/11217121/how-to-manage-memory-with-texture-in-opengl
void ViewerUtils::ConvertMatToGL(const cv::Mat &src, unsigned int *texID)
{
    if (src.empty() == true)
        return;

    glDeleteTextures(1, texID);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, texID);
    glBindTexture(GL_TEXTURE_2D, *texID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    if (src.channels() == 4)
    {
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            src.cols, src.rows,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            src.ptr<unsigned char>());
    }
    else if (src.channels() == 3)
    {
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            src.cols, src.rows,
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            src.ptr<unsigned char>());
    }
    else if (src.channels() == 1)
    {
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RED,
            src.cols, src.rows,
            0,
            GL_RED,
            GL_UNSIGNED_BYTE,
            src.ptr<unsigned char>());
    }
    else
    {
        std::cerr << "convertion from cv::Mat to GL texture failed!";
    }
}
