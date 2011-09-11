#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.imgproc import Scale, Interpolation
video_cap = VideoCapture(video_device=0, width=640, height=480)
fps = FPSDrawer()
factor = 0.1
scale_down = Scale(factor=factor, interpolation=Interpolation.AREA)
scale_up = Scale(factor=1 / factor, interpolation=Interpolation.LANCZOS4)

plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> scale_down['image'],
              scale_down['image'] >> scale_up['image'],
              scale_up['image'] >> fps['image'],
              fps['image'] >> imshow(name='Rescaled')['image'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture a video from the device and display it.',locals=vars())
