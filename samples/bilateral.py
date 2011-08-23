#!/usr/bin/env python
#
# Simple vid cap
#
import ecto
from ecto_opencv import highgui, calib, imgproc


plasm = ecto.Plasm()
video_cap = highgui.VideoCapture(video_device=0)

fps = highgui.FPSDrawer()

video_display = highgui.imshow('imshow',
                                name='video_cap', waitKey=10,
                                triggers=dict(save=ord('s')),
                               )

saver = ecto.If(cell=highgui.ImageSaver("saver", filename_format='bilateral_%05d.jpg',
                                   start=1))

bl_begin = None
bl_end = None

#make chain of bilateral filters.
for i in range(1, 10):
    next = imgproc.BilateralFilter(d= -1, sigmaColor=10, sigmaSpace=5)
    if bl_begin == None: # beginning case
        bl_begin = next
        bl_end = next
        continue
    plasm.connect(bl_end[:] >> next[:])
    bl_end = next

plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> bl_begin[:],
              bl_end[:] >> (video_display['input'], saver['image']),
              video_display['save'] >> saver['__test__'],
              )



if __name__ == '__main__':
    ecto.view_plasm(plasm)
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
