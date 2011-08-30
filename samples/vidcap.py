#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer

video_cap = VideoCapture(video_device=0, width=640, height=480)
fps = FPSDrawer()
video_display = imshow(name='video_cap', waitKey=2)

plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> video_display['input'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture a video from the device and display it.')
