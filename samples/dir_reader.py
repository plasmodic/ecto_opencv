#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageReader
import os

#this will read all images on the user's Desktop
images = ImageReader(path=os.path.expanduser('~/Desktop'))

#this is similar to a slide show... Wait for half a second
imshow = imshow(name='image', waitKey=500)

plasm = ecto.Plasm()
plasm.connect(images['image'] >> imshow['image'])

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Displays images from the user\'s Desktop.')