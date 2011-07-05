#!/bin/python
import ecto
from ecto_opencv import highgui, calib, imgproc

plasm = ecto.Plasm()
sched = ecto.schedulers.Threadpool(plasm)

video_cap = highgui.VideoCapture(video_device=0)
fps = highgui.FPSDrawer()
video_display = highgui.imshow('imshow',name='video_cap',waitKey=10,autoSize=True)

plasm.connect( video_cap['image'] >> fps['image'],
               fps['image'] >> video_display['input']
            )

ecto.view_plasm(plasm)
sched.execute()
