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
rgb2gray = imgproc.cvtColor (flag=7)
gaussian = imgproc.GaussianBlur(sigma=2.0)
circle_drawer = calib.CircleDrawer()
circle_drawer2 = calib.CircleDrawer()
pong = calib.PingPongDetector(dp=2, maxRadius=500, minRadius=1, param1=200, param2=100, minDist=20)

print pong.__doc__
show_circles = highgui.imshow("show circles", name="Circles", waitKey=10)

plasm.connect(video["image"] >> (rgb2gray["input"], circle_drawer["image"]),
              rgb2gray["out"] >> gaussian["input"],
              gaussian["out"] >> pong["image"],
              pong["circles"] >> circle_drawer["circles"],
              circle_drawer["image"] >> show_circles["input"]
              )
if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

sched = ecto.schedulers.Threadpool(plasm)
sched.execute(nthreads=8)

