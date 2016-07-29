/*
 * image_label.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: rdu
 */

#include "image_label.h"

#include <iostream>

using namespace srcl_ctrl;

ImageLabel::ImageLabel(QWidget *parent) :
    QWidget(parent),
    scaled_height_(0),
    scaled_width_(0),
	painter_height_(0),
	painter_width_(0),
	half_padded_height_(0),
	half_padded_width_(0)
{
}

void ImageLabel::paintEvent(QPaintEvent *event) {
    QWidget::paintEvent(event);

    if (pix_.isNull())
        return;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QSize pixSize = pix_.size();
    pixSize.scale(event->rect().size(), Qt::KeepAspectRatio);

    QPixmap scaledPix = pix_.scaled(pixSize,
                                   Qt::KeepAspectRatio,
                                   Qt::SmoothTransformation
                                   );

    // adjust painting area to be at the center
    /* reference: http://stackoverflow.com/questions/18959083/qpainter-drawimage-centeraligned */
    double widgetWidth = this->width();
    double widgetHeight = this->height();
    QRectF target(0, 0, widgetWidth, widgetHeight);

    double imageSizeWidth = static_cast<double>(scaledPix.width());
    double imageSizeHeight = static_cast<double>(scaledPix.height());
    QRectF source(0.0, 0.0, imageSizeWidth, imageSizeHeight);

    int deltaX = 0;
    int deltaY = 0;
    if(source.width() < target.width())
    	deltaX = target.width() - source.width();
    else
    	deltaX = source.width() - target.width();

    if(source.height() < target.height())
    	deltaY = target.height() - source.height();
    else
    	deltaY = source.height() - target.height();

    painter.translate(deltaX / 2, deltaY / 2);

    // store the scaled pixmap size
    scaled_height_ = scaledPix.height();
    scaled_width_ = scaledPix.width();

    // paint the pixmap
    painter.drawPixmap(QPoint(), scaledPix);

    painter_height_ = painter.device()->height();
    painter_width_ = painter.device()->width();

    half_padded_width_ = (painter_width_ - scaled_width_)/2;
    half_padded_height_ = (painter_height_ - scaled_height_)/2;
}

const QPixmap* ImageLabel::getPixmap() const {
    return &pix_;
}

void ImageLabel::setPixmap (const QPixmap &pixmap){
    pix_ = pixmap;

    scaled_height_ = pix_.height();
    scaled_width_ = pix_.width();
}

void ImageLabel::mousePressEvent( QMouseEvent* ev )
{
    if(ev->buttons() & Qt::LeftButton)
    {
        QPoint pt = this->mapFrom(this, ev->pos());

        // ignore clicking if clicked on non-map area
        if(pt.x() < half_padded_width_ || pt.x() > scaled_width_ + half_padded_width_
        		|| pt.y() < half_padded_height_ || pt.y() > scaled_width_  + half_padded_height_)
            return;

//        std::cout << "mouse clicked position: "
//        		<< pt.x() - half_padded_width_ << " , "
//        		<< pt.y() - half_padded_height_ << std::endl;

        double raw2scale_ratio = static_cast<double>(pix_.width())/static_cast<double>(scaled_width_);

        emit NewImagePositionClicked(pt.x() - half_padded_width_, pt.y() - half_padded_height_, raw2scale_ratio);
    }
}
