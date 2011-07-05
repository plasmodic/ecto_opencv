#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc

plasm = ecto.Plasm()
sched = ecto.schedulers.Threadpool(plasm)

video_cap = highgui.VideoCapture(video_device=0)
fps = highgui.FPSDrawer()
rgb2gray = imgproc.cvtColor('rgb -> gray',flag=7)

display_strand = ecto.Strand()
video_display = highgui.imshow('imshow',name='video_cap',waitKey=10,autoSize=True, strand=display_strand)

circle_detector = calib.PatternDetector(rows=7, cols=3, pattern_type="acircles", square_size=0.03)
circle_drawer = calib.PatternDrawer(rows=7, cols=3)
circle_display = highgui.imshow('Pattern show',name='Pattern',waitKey=-1,autoSize=True,strand=display_strand)

plasm.connect( video_cap['image'] >> (circle_drawer['input'],video_display['input']),
               circle_drawer['out'] >> fps['image'],
               fps['image'] >> circle_display['input'],
               video_cap['image'] >> rgb2gray['input'],
               rgb2gray['out'] >> circle_detector['input'],
               circle_detector['out','found'] >> circle_drawer['points','found'],
            )


ecto.view_plasm(plasm)
sched.execute()
