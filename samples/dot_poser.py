#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc

debug = True

plasm = ecto.Plasm()
rows = 7
cols = 3
square_size = 0.03 # in millis

pattern_show = highgui.imshow(name="pattern", waitKey=10, autoSize=True)
rgb2gray = imgproc.cvtColor(flag=7)
video = highgui.VideoCapture(video_device=0)
circle_detector = calib.PatternDetector(rows=rows, cols=cols, pattern_type="acircles", square_size=square_size)
circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
poser = calib.FiducialPoseFinder()
pose_drawer = calib.PoseDrawer()
camera_intrinsics = calib.CameraIntrinsics(camera_file="camera.yml")


plasm.connect(video, "image", rgb2gray, "input")
plasm.connect(rgb2gray, "out", circle_detector, "input")
plasm.connect(video, "image", circle_drawer, "input")
plasm.connect(circle_detector, "out", circle_drawer, "points")
plasm.connect(circle_detector, "found", circle_drawer, "found")
plasm.connect(camera_intrinsics, "K", poser, "K")
plasm.connect(circle_detector, "out", poser, "points")
plasm.connect(circle_detector, "ideal", poser, "ideal")
plasm.connect(circle_detector, "found", poser, "found")
plasm.connect(poser, "R", pose_drawer, "R")
plasm.connect(poser, "T", pose_drawer, "T")
plasm.connect(circle_drawer, "out", pose_drawer, "image")
plasm.connect(camera_intrinsics, "K", pose_drawer, "K")
plasm.connect(pose_drawer, "output", pattern_show, "input")


if debug:
    ecto.view_plasm(plasm)

sched = ecto.schedulers.Threadpool(plasm)
sched.execute(8)
    

