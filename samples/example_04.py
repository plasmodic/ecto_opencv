#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc

plasm = ecto.Plasm()
sched = ecto.schedulers.Singlethreaded(plasm)

video_cap = highgui.VideoCapture(video_device=0)

fps = highgui.FPSDrawer()

rgb2gray = imgproc.cvtColor('rgb -> gray',
                            flag=7)

display_strand = ecto.Strand()

gray_display = highgui.imshow('gray show',
                              name='gray', waitKey=-1, autoSize=True, strand=display_strand)

video_display = highgui.imshow('imshow',
                               name='video_cap', waitKey=10, autoSize=True, strand=display_strand)

plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> video_display['input'],
              video_cap['image'] >> rgb2gray['input'],
              rgb2gray['out'] >> gray_display['input'],
              )

ecto.view_plasm(plasm)
sched.execute()
