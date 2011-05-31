#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui,calib,imgproc

debug = True

plasm = ecto.Plasm()

imshow = highgui.imshow(name="image", waitKey=20, autoSize=True)
images = highgui.VideoCapture(video_device=1)


plasm.connect(images, "out", imshow, "input")

ecto.view_plasm(plasm)

while(imshow.outputs.out != 27):
    x = plasm.execute()
    if x :
        break;
    

