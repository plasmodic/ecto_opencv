#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.imgproc import Canny, RGB2GRAY, cvtColor

video_cap = VideoCapture(video_device=0, width=640, height=480)
fps = FPSDrawer()
canny = Canny()
gray = cvtColor(flag=RGB2GRAY)
plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> gray[:],
              gray[:] >> canny['image'],
              canny['image'] >> fps['image'],
              fps['image'] >> imshow(name='Canny Image')['image'],
              video_cap['image'] >> imshow(name='Original Image')['image'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture a video from the device and display it.')
