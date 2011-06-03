#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc

import os

debug = False

plasm = ecto.Plasm()

#this will read all images on the user's Desktop
images = highgui.ImageReader(path=os.path.expanduser("~/Desktop"))

#this is similar to a slide show... Wait for half a second
imshow = highgui.imshow(name="image", waitKey=500, autoSize=True)

plasm.connect(images, "out", imshow, "input")

if debug:
    ecto.view_plasm(plasm)

while(imshow.outputs.out not in (27, ord('q'))):
    x = plasm.execute()
    if x :
        break;
    
