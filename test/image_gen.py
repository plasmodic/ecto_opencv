#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow
from ecto_opencv.opencv_test import ImageGen, ImageCmp, ImageDelay

class DelayBox(ecto.BlackBox):
    d3 = ImageDelay
    img_gen = ImageGen
    def declare_io(self, p, i, o):
        o.forward_all('d3')
        o.forward('gen','img_gen','image')
    def connections(self):
        img_gen = self.img_gen
        d3 = self.d3
        d1,d2 = ImageDelay(),ImageDelay()
        graph = [img_gen[:] >> d1[:],
                d1[:] >> d2[:],
                d2[:] >> d3[:],
               ]
        return graph

plasm = ecto.Plasm()

d1,d2,d3 = ImageDelay(),ImageDelay(),ImageDelay()
img_gen = ImageGen()[:]
img_cmp = ImageCmp()
plasm.connect(
              img_gen >> d1[:],
              d1[:] >> d2[:],
              d2[:] >> d3[:],
              d3[:] >> img_cmp['image1'],
              img_gen >> img_cmp['image2'],
              )

#ecto.view_plasm(plasm)
sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute(niter=10)
print sched.stats()
