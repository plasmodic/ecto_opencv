#!/usr/bin/env python
import ecto
import ecto_opencv
from ecto_opencv import highgui, opencv_test
import os.path

try:
    saver = highgui.ImageSaver(filename_format='img.png')
except RuntimeError, e:
    assert 'boost::too_many_args:' in str(e)

saver = highgui.ImageSaver(filename_format='img_%05d.png')

plasm = ecto.Plasm()
plasm.connect(opencv_test.ImageGen()['image'] >> saver['image'])
sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute(niter=1)

assert os.path.isfile('img_00000.png')
stats =  os.stat('img_00000.png')
print stats
assert stats.st_size > 1000
os.remove('img_00000.png')
