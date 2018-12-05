import math
import cairo

from enum import Enum


class CanvasType(Enum):
    IMAGE = 1
    SVG = 2
    PS = 3
    PDF = 4


class CairoCanvas:
    '''
    Description: A canvas whose coordinates are mapped to [0,1]. 

    Note: The canvas is always created as a square to keep the right aspect ratio and only
        the part spcified by [0, width] * [0, height] are visible to user. 
        Internally the size of the square is equal to either width or height, whichever is 
        larger, which also means the point at [1,1] may be outside the visible part of canvas.            
    
    Author: Ruixiang Du
    '''

    def __init__(self, width=600, height=400, name="cairo_default", add_boundary=True, type=CanvasType.IMAGE):
        self.dwidth = width
        self.dheight = height

        ssize = width
        if width < height:
            ssize = height
        self.width = ssize
        self.height = ssize
        self.name = name
        self.type = type

        self.add_suffix_to_name()
        self.setup_canvas()

        if add_boundary == True:
            self.draw_reference_boundary()

    def setup_canvas(self):
        self.surface = self.create_surface(self.name, self.width, self.height)
        self.context = cairo.Context(self.surface)
        self.context.scale(self.width, self.height)  # Normalizing the canvas

    def add_suffix_to_name(self):
        if self.type == CanvasType.IMAGE:
            self.name = self.name + ".png"
        elif self.type == CanvasType.SVG:
            self.name = self.name + ".svg"
        elif self.type == CanvasType.PS:
            self.name = self.name + ".eps"
        elif self.type == CanvasType.PDF:
            self.name = self.name + ".pdf"

    def create_surface(self, name, width, height):
        if self.type == CanvasType.IMAGE:
            surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height)
        elif self.type == CanvasType.SVG:
            surface = cairo.SVGSurface(name, width, height)
        elif self.type == CanvasType.PS:
            surface = cairo.PSSurface(name, width, height)
            surface.set_eps(True)
        elif self.type == CanvasType.PDF:
            surface = cairo.PDFSurface(name, width, height)
        return surface

    def get_context(self):
        return self.context

    def translate_rotate(self, dx, dy, dtheta):
        self.context.translate(dx, dy)
        self.context.rotate(dtheta)

    def rotate_translate(self, dx, dy, dtheta):
        self.context.rotate(dtheta)
        self.context.translate(dx, dy)
        
    def enable_clear_mode(self):
        self.context.set_operator(cairo.Operator.CLEAR)
    
    def disable_clear_mode(self):
        self.context.set_operator(cairo.Operator.OVER)

    def draw_reference_boundary(self):
        ctx = self.context
        lwidth = 0.005
        ctx.rectangle(0, 0, self.dwidth/self.width, self.dheight/self.height)
        ctx.set_line_width(lwidth)
        ctx.set_source_rgb(0, 0, 0)
        ctx.stroke()

    def crop_surface(self):
        # Reference: https://www.programcreek.com/python/example/2108/cairo.ImageSurface
        # cropped = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height)
        csurface = self.create_surface(self.name, self.dwidth, self.dheight)
        cropped_context = cairo.Context(csurface)
        cropped_context.rectangle(0, 0, self.dwidth, self.dheight)
        cropped_context.set_source_surface(self.surface, 0, 0)
        cropped_context.fill()
        self.surface = csurface

    def save_to_file(self):
        # crop surface to desired size
        self.crop_surface()

        # only need to explicitly call save function with image surface
        if self.type == CanvasType.IMAGE:
            self.surface.write_to_png(self.name)  # Output to PNG
