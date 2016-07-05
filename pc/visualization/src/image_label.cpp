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
    scaled_height(0),
    scaled_width(0)
{
}

void ImageLabel::paintEvent(QPaintEvent *event) {
    QWidget::paintEvent(event);

    if (pix.isNull())
        return;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QSize pixSize = pix.size();
    pixSize.scale(event->rect().size(), Qt::KeepAspectRatio);

    QPixmap scaledPix = pix.scaled(pixSize,
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
    scaled_height = scaledPix.height();
    scaled_width = scaledPix.width();

    // paint the pixmap
    painter.drawPixmap(QPoint(), scaledPix);
}

const QPixmap* ImageLabel::getPixmap() const {
    return &pix;
}

void ImageLabel::setPixmap (const QPixmap &pixmap){
    pix = pixmap;

    scaled_height = pix.height();
    scaled_width = pix.width();
}

void ImageLabel::mousePressEvent( QMouseEvent* ev )
{
    if(ev->buttons() & Qt::LeftButton)
    {
        QPoint pt = this->mapFrom(this, ev->pos());

        if(pt.x() > scaled_width || pt.y() > scaled_height)
            return;

//        std::cout << "original size: " << pix.width() << " , " << pix.height() << std::endl;
//        std::cout << "scaled size: " << scaled_width << " , " << scaled_height << std::endl;
        std::cout << "mouse clicked position: " << pt.x() << " , " << pt.y() << std::endl;

        double raw2scale_ratio = static_cast<double>(pix.width())/static_cast<double>(scaled_width);

        emit NewImagePositionClicked(pt.x(), pt.y(), raw2scale_ratio);
    }
}
