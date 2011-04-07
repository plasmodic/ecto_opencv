#!/bin/python
import ecto

from ecto_opencv import highgui,calib,imgproc

debug = True

plasm = ecto.Plasm()
rows = 9
cols = 3
square_size = 40 # in millis
pattern_show = ecto.make(highgui.imshow, name="pattern", waitKey=10, autoSize=True)
rgb2gray = ecto.make(imgproc.cvtColor, flag=7)
video = ecto.make(highgui.VideoCapture, video_device=0)
circle_detector = ecto.make(calib.PatternDetector, rows=rows, cols=cols)
circle_drawer = ecto.make(calib.PatternDrawer, rows=rows, cols=cols)
camera_calibrator = ecto.make(calib.CameraCalibrator, rows=rows, cols=cols, square_size=square_size)
ecto.print_module_doc(camera_calibrator)
plasm.connect(video, "out", rgb2gray, "in")
plasm.connect(rgb2gray, "out", circle_detector, "in")
plasm.connect(video, "out", circle_drawer, "in")
plasm.connect(circle_detector, "out", circle_drawer, "points")
plasm.connect(circle_detector, "found", circle_drawer, "found")
plasm.connect(circle_drawer, "out", pattern_show, "in")
plasm.connect(video, "out", camera_calibrator, "image")
plasm.connect(circle_detector, "out", camera_calibrator,"points")
plasm.connect(circle_detector, "found", camera_calibrator, "found")

while(pattern_show.o.out.get() != 27):
    plasm.mark_dirty(video)
    #plasm.go(lazer_show)
    plasm.go(pattern_show)
    plasm.go(camera_calibrator)

    

