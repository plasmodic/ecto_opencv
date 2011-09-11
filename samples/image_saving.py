#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, ImageSaver

video_cap = VideoCapture(video_device=0)
fps = FPSDrawer()
video_display = imshow(name='video_cap', triggers=dict(save=ord('s')))
saver = ecto.If(cell=ImageSaver('saver', filename_format='ecto_image_%05d.jpg',
                                   start=1))

plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> video_display['image'],
              video_display['save'] >> saver['__test__'],
              fps['image'] >> saver['image'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Save images from video stream.')
