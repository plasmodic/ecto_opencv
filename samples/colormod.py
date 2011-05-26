#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui,calib,imgproc, line_mod

debug = True

plasm = ecto.Plasm()  #Constructor for Plasm
video = highgui.VideoCapture(video_device=0)
coded_color = highgui.imshow(name="coded_color", waitKey=10, autoSize=True)

ecto.print_module_doc(video)
ecto.print_module_doc(coded_color)

plasm.connect(video, "out", coded_color, "input")

ecto.view_plasm(plasm)

while(coded_color.outputs.out != 27):
    plasm.execute()
    

