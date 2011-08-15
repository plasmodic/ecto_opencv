#!/usr/bin/env python
import ecto
from ecto_opencv import imgproc, highgui, features2d

plasm = ecto.Plasm()

video = highgui.VideoCapture(video_device=0)
orb_m = features2d.ORB(n_features=2500)
draw_kpts = features2d.DrawKeypoints()
orb_display = highgui.imshow('orb display', name="ORB", waitKey=5, autoSize=True)
rgb2gray = imgproc.cvtColor (flag=imgproc.Conversion.RGB2GRAY)
fps = highgui.FPSDrawer()

plasm.connect(video["image"] >> rgb2gray ["input"],
                rgb2gray["out"] >> orb_m["image"],
                orb_m["kpts"] >> draw_kpts["kpts"],
                video["image"] >> draw_kpts["input"],
                draw_kpts["output"] >> fps[:],
                fps[:] >> orb_display["input"],
              )

if __name__ == '__main__':
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()