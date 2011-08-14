#!/usr/bin/env python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc
import os

debug = True

plasm = ecto.Plasm()

#this will read all images on the user's Desktop
images = highgui.ImageReader("image reader",path=os.path.expanduser("/home/bradski/code/data/set01/images"))
masks = highgui.ImageReader("mask reader",path=os.path.expanduser("/home/bradski/code/data/set01/masks"))

#this is similar to a slide show... Wait forever for a key
image_display = highgui.imshow("image display",name="image", waitKey=0, autoSize=True)
mask_display = highgui.imshow("mask display", name="mask", waitKey=-1, autoSize=True)

plasm.connect(images["out"] >> image_display['input'],
              masks["out"] >> mask_display['input'],
              )

if debug:
    ecto.view_plasm(plasm)

while(image_display.outputs.out not in (ord('q'),27) ):
    x = plasm.execute()
    if x :
        break;