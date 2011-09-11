#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.features2d import ORB, DrawKeypoints
from ecto_opencv.imgproc import cvtColor, Conversion

video = VideoCapture(video_device=0)
orb_m = ORB(n_features=2500)
draw_kpts = DrawKeypoints()
rgb2gray = cvtColor (flag=Conversion.RGB2GRAY)
fps = FPSDrawer()

plasm = ecto.Plasm()
plasm.connect(video['image'] >> rgb2gray ['image'],
                rgb2gray['image'] >> orb_m['image'],
                orb_m['keypoints'] >> draw_kpts['keypoints'],
                video['image'] >> draw_kpts['image'],
                draw_kpts['image'] >> fps[:],
                fps[:] >> imshow('orb display', name='ORB')['image'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Computes the ORB feature and descriptor on a video stream.')
