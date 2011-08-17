#!/usr/bin/env python
import ecto_opencv
from ecto_opencv import highgui

try:
    saver = highgui.ImageSaver(filename_format='img.png')
except RuntimeError, e:
    assert 'boost::too_many_args:' in str(e)

saver = highgui.ImageSaver(filename_format='img_%05d.png')
saver.configure()
saver.process()

