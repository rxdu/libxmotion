#!/usr/bin/env python

import math
import cairo
import cairo_canvas as cc

def draw_ramp():
    canvas = cc.CairoCanvas(600, 400, "template", True, cc.CanvasType.IMAGE)
    ctx = canvas.get_context()
    
    ##################
    #   draw here
    ##################

    ctx.rectangle(0.05, 0.1, 0.9, 0.12)
    ctx.set_source_rgb(169/255.0, 169/255.0, 169/255.0)
    ctx.fill()

    ##################
        
    canvas.save_to_file()

def main():
    print("librav pycairo drawing: " + __file__)

    draw_ramp()


if __name__ == '__main__':
    main()
