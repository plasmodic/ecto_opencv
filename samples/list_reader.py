#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageReader
import sys

image_list = sys.argv[1:]
#this will read all images on the user's Desktop
images = ImageReader(file_list=ecto.list_of_strings(image_list))

#this is similar to a slide show... Wait for half a second
imshow = imshow(name='image')

plasm = ecto.Plasm()
plasm.connect(images['image'] >> imshow['image'])

ecto.schedulers.Singlethreaded(plasm).execute()

