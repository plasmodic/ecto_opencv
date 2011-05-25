#!/bin/python
import ecto
import cv_bp as opencv
from ecto_opencv import highgui,calib,imgproc

debug = True

plasm = ecto.Plasm()
rows = 11
cols = 4
square_size = 40 # in millis

pattern_show = highgui.imshow(name="pattern", waitKey=10, autoSize=True)
rgb2gray = imgproc.cvtColor(flag=opencv.CV_)
video = highgui.VideoCapture(video_device=0)
circle_detector = calib.PatternDetector(rows=rows, cols=cols,pattern_type="acircles")
circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
camera_calibrator = calib.CameraCalibrator(rows=rows, cols=cols, square_size=square_size)

ecto.print_module_doc(camera_calibrator)

plasm.connect(video, "out", rgb2gray, "input")
plasm.connect(rgb2gray, "out", circle_detector, "input")
plasm.connect(video, "out", circle_drawer, "input")
plasm.connect(circle_detector, "out", circle_drawer, "points")
plasm.connect(circle_detector, "found", circle_drawer, "found")
plasm.connect(circle_drawer, "out", pattern_show, "input")
plasm.connect(video, "out", camera_calibrator, "image")
plasm.connect(circle_detector, "out", camera_calibrator,"points")
plasm.connect(circle_detector, "found", camera_calibrator, "found")

while(pattern_show.outputs.out != 27):
    plasm.execute()
    

