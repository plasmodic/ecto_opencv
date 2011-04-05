#!/bin/python
import ecto
from ecto.doc import print_module_doc, graphviz
from ecto_opencv import highgui, imgproc, lazer, calib
debug = True

plasm = ecto.Plasm()

splitter = ecto.make(imgproc.ChannelSplitter)
sl_drawer = ecto.make(lazer.ScanLineDrawer)
laser_detector = ecto.make(lazer.LaserDetector)
imshowL = ecto.make(highgui.imshow, name="L", waitKey=10, autoSize=True)
imshowA = ecto.make(highgui.imshow, name="A", waitKey= -1, autoSize=True)
imshowB = ecto.make(highgui.imshow, name="B", waitKey= -1, autoSize=True)
pattern_show = ecto.make(highgui.imshow, name="pattern", waitKey= -1, autoSize=True)
lazer_show = ecto.make(highgui.imshow, name="lazer", waitKey= -1, autoSize=True)
rgb2gray = ecto.make(imgproc.cvtColor, flag=7)
bgr2lab = ecto.make(imgproc.cvtColor, flag=44)
print_module_doc(rgb2gray)
video = ecto.make(highgui.VideoCapture, video_device=0)
camera_calibrator = ecto.make(calib.CameraCalibrator, rows=11, cols=4)
print_module_doc(camera_calibrator)

plasm.connect(video, "out", rgb2gray, "in")
plasm.connect(video, "out", bgr2lab, "in")
plasm.connect(bgr2lab, "out", splitter, "in")
plasm.connect(splitter, "out_0", imshowL, "in")
plasm.connect(splitter, "out_1", imshowA, "in")
plasm.connect(splitter, "out_2", imshowB, "in")
plasm.connect(splitter, "out_1", sl_drawer, "in")
#plasm.connect(splitter, "green", lazer_show, "in")
plasm.connect(sl_drawer, "out", lazer_show, "in")

graphviz(plasm)
ecto.view_plasm(plasm)

while(imshowL.o.out.get() != 27):
    plasm.markDirty(video)
    plasm.go(lazer_show)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(imshowA)
    plasm.go(imshowB)
    plasm.go(imshowL)

    

