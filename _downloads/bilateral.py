#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, FPSDrawer, imshow, ImageSaver
from ecto_opencv.imgproc import BilateralFilter

video_cap = VideoCapture(video_device=0)

fps = FPSDrawer()

video_display = imshow('imshow',
                      name='video_cap', triggers=dict(save=ord('s')),
                      )

saver = ecto.If(cell=ImageSaver('saver', filename_format='bilateral_%05d.jpg',
                                   start=1))

bl_begin = None
bl_end = None

plasm = ecto.Plasm()
#Build up a chain of bilateral filters.
for i in range(1, 10):
    next = BilateralFilter(d= -1, sigmaColor=10, sigmaSpace=5)
    if bl_begin == None: # beginning case
        bl_begin = next
        bl_end = next
        continue
    plasm.connect(bl_end[:] >> next[:])
    bl_end = next

plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> bl_begin[:],
              bl_end[:] >> (video_display['image'], saver['image']),
              video_display['save'] >> saver['__test__'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Run a chain of bilateral smoothing filters on a video stream.')

