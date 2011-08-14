#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc
import os

#this will read all images on the user's Desktop
images = highgui.ImageReader(path=os.path.expanduser("~/Desktop"))

#this is similar to a slide show... Wait for half a second
imshow = highgui.imshow(name="image", waitKey=500, autoSize=True)

plasm = ecto.Plasm()
plasm.connect(images, "out", imshow, "input")

if __name__ == '__main__':
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
