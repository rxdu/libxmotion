/*
 * cairo_context.cpp
 *
 * Created on: Mar 04, 2021 17:01
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/details/cairo_context.hpp"

#include <iostream>

namespace xmotion {
namespace swviz {
CairoContext::CairoContext(uint32_t width, uint32_t height,
                           bool normalize_coordinate)
    : width_(width),
      height_(height),
      normalize_coordinate_(normalize_coordinate) {
  // create cairo context
  CreateSurface();
  GenGlTexture();
}

CairoContext::~CairoContext() {
  // destroy resource
  if (gl_texture_created_) glDeleteTextures(1, &image_texture_);
  cairo_surface_destroy(surface_);
  cairo_destroy(cr_);
}

void CairoContext::Resize(uint32_t width, uint32_t height) {
  // no need to resize
  if (width_ == width && height_ == height) return;

  // destroy old surface and context
  cairo_surface_destroy(surface_);
  cairo_destroy(cr_);

  // create new surface and context
  width_ = width;
  height_ = height;
  CreateSurface();
}

float CairoContext::GetAspectRatio() const { return aspect_ratio_; }

void CairoContext::CreateSurface() {
  surface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width_, height_);
  if (cairo_surface_status(surface_) != CAIRO_STATUS_SUCCESS) {
    throw std::runtime_error(
        "[ERROR] create_cairo_context() - Couldn't create surface\n");
  }

  cr_ = cairo_create(surface_);
  if (cairo_status(cr_) != CAIRO_STATUS_SUCCESS) {
    throw std::runtime_error(
        "[ERROR] create_cairo_context() - Couldn't create context\n");
  }

  if (normalize_coordinate_) {
    // auto min = std::min(width_, height_);
    cairo_scale(cr_, height_, height_);
  }

  aspect_ratio_ = static_cast<float>(width_) / height_;
}

void CairoContext::GenGlTexture() {
  // create and setup OpenGL texture
  glGenTextures(1, &image_texture_);
  glBindTexture(GL_TEXTURE_2D, image_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // the following two are required on WebGL for non power-of-two textures
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  gl_texture_created_ = true;
}

void CairoContext::PushScale(double sx, double sy) {
  scaler_stack_.push({sx, sy});
  cairo_scale(cr_, sx, sy);
}

void CairoContext::PopScale() {
  if (scaler_stack_.empty()) return;
  auto scaler = scaler_stack_.top();
  cairo_scale(cr_, 1.0f / scaler.x, 1.0f / scaler.y);
  scaler_stack_.pop();
}

GLuint CairoContext::RenderToGlTexture() {
  // bind texture
  glBindTexture(GL_TEXTURE_2D, image_texture_);

  // convert surface to OpenGL texture
  unsigned char* data = cairo_image_surface_get_data(surface_);
  glTexImage2D(GL_TEXTURE_2D, 0, 4, width_, height_, 0, GL_BGRA,
               GL_UNSIGNED_BYTE, data);

  // unbind texture
  glBindTexture(GL_TEXTURE_2D, 0);

  return image_texture_;
}
}  // namespace swviz
}  // namespace xmotion