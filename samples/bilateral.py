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
                               name='video_cap', waitKey=10)

bl = imgproc.BilateralFilter(d = -1, sigmaColor=10, sigmaSpace=5)

plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> bl[:],
              bl[:] >> video_display['input'],
              )


if __name__ == '__main__':
    ecto.view_plasm(plasm)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

