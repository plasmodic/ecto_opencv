#!/usr/bin/env python
import ecto
import ecto_opencv
from ecto_opencv import highgui, opencv_test
import ecto_opencv.cv_bp
import os.path

saver = highgui.MatWriter(filename='x.yaml')

plasm = ecto.Plasm()
plasm.connect(opencv_test.MatGen()['mat'] >> saver['mat'])
sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute(niter=1)

assert os.path.isfile('x.yaml')
reader = highgui.MatReader(filename='x.yaml')

plasm = ecto.Plasm()
plasm.connect(reader['mat'] >> highgui.MatPrinter(name='x')[:])
sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute(niter=1)

assert reader.outputs.mat.rows != 0
