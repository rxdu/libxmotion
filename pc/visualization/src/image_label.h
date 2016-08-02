/*
 * image_label.h
 *
 *  Created on: Apr 27, 2016
 *      Author: rdu
 *
 *  Source:
 *      http://stackoverflow.com/questions/14107144/how-do-i-make-an-image-resize-to-scale-in-qt
 *      http://stackoverflow.com/questions/16279762/qt-mousemoveevent-only-when-left-mouse-button-is-pressed
 *
 */

#ifndef SRCL_ROS_QUAD_NAV_INCLUDE_QUAD_NAV_IMAGE_LABEL_H_
#define SRCL_ROS_QUAD_NAV_INCLUDE_QUAD_NAV_IMAGE_LABEL_H_

#include <cstdint>

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include <QMouseEvent>

namespace srcl_ctrl {

class ImageLabel : public QWidget
{
	Q_OBJECT

public:
	explicit ImageLabel(QWidget *parent = 0);
	const QPixmap* getPixmap() const;

public slots:
	void setPixmap(const QPixmap&);

protected:
	void paintEvent(QPaintEvent *);

signals:
    void MapImageClicked(long x, long y, double raw2scale_ratio, uint8_t btn_flag);

private:
	QPixmap pix_;
    long scaled_height_;
    long scaled_width_;
    long painter_height_;
    long painter_width_;
    long half_padded_height_;
    long half_padded_width_;

    void mousePressEvent(QMouseEvent* ev);
};

}

#endif /* SRCL_ROS_QUAD_NAV_INCLUDE_QUAD_NAV_IMAGE_LABEL_H_ */
