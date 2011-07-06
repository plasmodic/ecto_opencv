#!/usr/bin/env python
# Vid cap and gray conversion
import ecto
from ecto_opencv import highgui, calib, imgproc

plasm = ecto.Plasm()
sched = ecto.schedulers.Singlethreaded(plasm)

video_cap = highgui.VideoCapture(video_device=0)

fps = highgui.FPSDrawer()

video_display = highgui.imshow('imshow', 
                               name='video_cap', waitKey=10, autoSize=True)

rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)

gray_display = highgui.imshow('gray show',
                              name='gray', waitKey=-1, autoSize=True)

plasm.connect(video_cap['image'] >> rgb2gray['input'],
              video_cap['image'] >> fps['image'],
              fps['image'] >> video_display['input'],
              rgb2gray['out'] >> gray_display['input'],
              )

ecto.view_plasm(plasm)
sched.execute()
