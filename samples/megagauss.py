#!/usr/bin/env python
import ecto
from ecto_opencv import imgproc, highgui, features2d, calib
import time, signal
#import orb as imgproc
signal.signal(signal.SIGINT, signal.SIG_DFL)


debug = True
#debug = True

plasm = ecto.Plasm()

video = highgui.VideoCapture(video_device=0)

k = 25
g = imgproc.GaussianBlur(sigma=5)
plasm.connect(video['image'] >> g['input'])

for x in range(k):
    nextg = imgproc.GaussianBlur(sigma=5)
    plasm.connect(g['out'] >> nextg['input'])
    g = nextg

show_circles = highgui.imshow("megagauss", name="megagauss", waitKey=10)
plasm.connect(g['out'] >> show_circles['input'])

if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

#sched = ecto.schedulers.Singlethreaded(plasm)
#sched.execute()

sched = ecto.schedulers.Threadpool(plasm)
sched.execute(nthreads=8)

