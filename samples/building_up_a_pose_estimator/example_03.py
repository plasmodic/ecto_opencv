#!/usr/bin/env python
#
# add circle detection
#
import ecto
from ecto_opencv import highgui, calib, imgproc

plasm = ecto.Plasm()
sched = ecto.schedulers.Singlethreaded(plasm)

video_cap = highgui.VideoCapture(video_device=0)

fps = highgui.FPSDrawer()

video_display = highgui.imshow('imshow',
                               name='video_cap', waitKey=2, maximize=False)

rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)

circle_detector = calib.PatternDetector(rows=5, cols=3,
                                        pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                        square_size=0.04)

circle_drawer = calib.PatternDrawer(rows=5, cols=3)
circle_display = highgui.imshow('Pattern show', name='Pattern', waitKey= -1, maximize=False)

plasm.connect(video_cap['image'] >> rgb2gray['input'],
              video_cap['image'] >> (circle_drawer['input'],
                                     video_display['input']),
              fps['image'] >> circle_display['input'],
              circle_drawer['out'] >> fps['image'],
              
              rgb2gray['out'] >> circle_detector['input'],
              circle_detector['out', 'found'] >> circle_drawer['points', 'found'],
              )


ecto.view_plasm(plasm)
sched.execute()
