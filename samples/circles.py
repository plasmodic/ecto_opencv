#!/usr/bin/env python
import ecto
from ecto_opencv import imgproc, highgui, features2d, calib



video = highgui.VideoCapture("MyVidCapNode", video_device=0)
rgb2gray = imgproc.cvtColor (flag=imgproc.CV_RGB2GRAY)
gaussian = imgproc.GaussianBlur(sigma=2.0)
circle_drawer = calib.CircleDrawer()
circle_drawer2 = calib.CircleDrawer()
pong = calib.PingPongDetector(dp=2, maxRadius=500, minRadius=1, param1=200, param2=100, minDist=20)
show_circles = highgui.imshow("show circles", name="Circles", waitKey=10)

plasm = ecto.Plasm()
plasm.connect(video["image"] >> (rgb2gray["input"], circle_drawer["image"]),
              rgb2gray["out"] >> gaussian["input"],
              gaussian["out"] >> pong["image"],
              pong["circles"] >> circle_drawer["circles"],
              circle_drawer["image"] >> show_circles["input"]
              )

if __name__ == '__main__':
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

