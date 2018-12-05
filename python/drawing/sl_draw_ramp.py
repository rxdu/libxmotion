#!/usr/bin/env python

import math
import cairo
import cairo_canvas as cc


def draw_ramp():
    canvas = cc.CairoCanvas(600, 300, "ramp_example",
                            True, cc.CanvasType.IMAGE)
    ctx = canvas.get_context()

    ##################
    #   draw here
    ##################

    ctx.rectangle(0.05, 0.1, 0.9, 0.12)
    ctx.set_source_rgb(169/255.0, 169/255.0, 169/255.0)
    ctx.fill()

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.05, 0.11)
    ctx.line_to(0.95, 0.11)
    ctx.stroke()

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.05, 0.21)
    ctx.line_to(0.95, 0.21)
    ctx.stroke()

    #---------------------------------#

    dx = 0.15
    dy = 0.355
    dtheta = math.radians(-20)

    canvas.translate_rotate(dx, dy, dtheta)
    ctx.set_source_rgb(169/255.0, 169/255.0, 169/255.0)
    ctx.rectangle(0.0, 0.0, 0.7, 0.1)
    ctx.fill()

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.0, 0.01)
    ctx.line_to(0.45, 0.01)
    ctx.stroke()

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.0, 0.09)
    ctx.line_to(0.67, 0.09)
    ctx.stroke()

    #---------------------------------#

    canvas.reverse_translate_rotate(dx, dy, dtheta)

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.05, 0.21)
    ctx.line_to(0.95, 0.21)
    ctx.stroke()

    ctx.set_source_rgb(169/255.0, 169/255.0, 169/255.0)
    ctx.set_line_width(0.008)
    ctx.move_to(0.58, 0.21)
    ctx.line_to(0.802, 0.21)
    ctx.stroke()

    red_car_suf = cairo.ImageSurface.create_from_png("resource/red_car.png")
    width = red_car_suf.get_width()
    height = red_car_suf.get_height()
    
    ctx.rectangle(0, 0, 250, 250)
    ctx.move_to(0,0)
    ctx.set_source_surface(red_car_suf, width/2, height/2)
    ctx.fill()
    # ctx.clip()
    # ctx.paint()
    # ctx.restore()

    ##################

    canvas.save_to_file()


def main():
    print("librav pycairo drawing: " + __file__)

    draw_ramp()


if __name__ == '__main__':
    main()
