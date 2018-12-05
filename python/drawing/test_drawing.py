#!/usr/bin/env python

import cairo_canvas as cc
import math
import cairo


def test_canvas():
    canvas = cc.CairoCanvas(600, 400, "test_canvas", True, cc.CanvasType.IMAGE)
    ctx = canvas.get_context()
    draw_something(ctx)
    canvas.save_to_file()

def test_image():
    print("test image")

    canvas = cc.CairoCanvas(800, 600, "test_image", True, cc.CanvasType.IMAGE)
    ctx = canvas.get_context()

    image = cairo.ImageSurface.create_from_png("resource/red_car.png")
    ctx.set_source_surface(image)
    ctx.paint()

    canvas.save_to_file()

def draw_something(ctx):

    ctx.rectangle(0.1, 0.2, 0.8, 0.2)
    ctx.set_source_rgb(169/255.0, 169/255.0, 169/255.0)
    ctx.fill()

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.1, 0.22)
    ctx.line_to(0.9, 0.22)
    ctx.stroke()

    ctx.set_source_rgb(1, 1, 1)
    ctx.set_line_width(0.005)
    ctx.move_to(0.1, 0.38)
    ctx.line_to(0.9, 0.38)
    ctx.stroke()

    ctx.translate(0.2, 0.55)
    # ctx.rotate(-20/180.0*math.pi)
    ctx.rotate(math.radians(-30))
    ctx.set_source_rgb(169/255.0, 169/255.0, 169/255.0)
    # ctx.rectangle(0.1, 0.5, 0.6, 0.15)
    ctx.rectangle(0.0, 0.0, 0.6, 0.15)
    ctx.fill()


def main():
    print("test drawing: " + __file__)

    # test_canvas()
    test_image()
    

if __name__ == '__main__':
    main()
