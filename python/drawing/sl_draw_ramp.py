#!/usr/bin/env python

import math
import cairo

def draw_ramp():

    WIDTH, HEIGHT = 800, 800

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, WIDTH, HEIGHT)
    # surface = cairo.PSSurface("eps_example.eps", WIDTH, HEIGHT)
    # surface.set_eps(True)
    ctx = cairo.Context(surface)

    ctx.scale(WIDTH, HEIGHT)  # Normalizing the canvas

    # pat = cairo.LinearGradient(0.0, 0.0, 0.0, 1.0)
    # pat.add_color_stop_rgba(1, 0.7, 0, 0, 0.5)  # First stop, 50% opacity
    # pat.add_color_stop_rgba(0, 0.9, 0.7, 0.2, 1)  # Last stop, 100% opacity

    # ctx.rectangle(0, 0, 1, 1)  # Rectangle(x0, y0, x1, y1)
    # ctx.set_source(pat)
    # ctx.fill()

    # ctx.translate(0.1, 0.1)  # Changing the current transformation matrix

    # ctx.move_to(0, 0)
    # # Arc(cx, cy, radius, start_angle, stop_angle)
    # ctx.arc(0.2, 0.1, 0.1, -math.pi / 2, 0)
    # ctx.line_to(0.5, 0.1)  # Line to (x,y)
    # # Curve(x1, y1, x2, y2, x3, y3)
    # ctx.curve_to(0.5, 0.2, 0.5, 0.4, 0.2, 0.8)
    # ctx.close_path()

    ctx.rectangle(0, 0, 1, 1)
    ctx.set_line_width(0.01)
    ctx.set_source_rgb(0, 0, 0)
    ctx.stroke()

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

    surface.write_to_png("example.png")  # Output to PNG


def main():
    print("selective listening drawing: " + __file__)

    draw_ramp()


if __name__ == '__main__':
    main()
