#!/usr/bin/env python

import ecto
from ecto.doc import print_module_doc,graphviz
from ecto_opencv import highgui
from ecto_opencv import imgproc

video = ecto.make(highgui.VideoCapture)
imshow = ecto.make(highgui.imshow,name="video",waitKey=10)
sobelShower = ecto.make(highgui.imshow,name="sobel",waitKey=-1)
grayShower = ecto.make(highgui.imshow,name="gray",waitKey=-1)

sobelX = ecto.make(imgproc.Sobel, x= 1, y = 0)
sobelY = ecto.make(imgproc.Sobel, x= 0, y = 1)

rgb2gray = ecto.make(imgproc.cvtColor, flag=7)

adder = ecto.make(imgproc.ImageAdder)
abs1 = ecto.make(imgproc.AbsNormalized)
abs2 = ecto.make(imgproc.AbsNormalized)

plasm = ecto.Plasm()

plasm.connect(video, "out", imshow, "in")
plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(rgb2gray, "out", sobelX, "in")
plasm.connect(rgb2gray, "out", sobelY, "in")
plasm.connect(rgb2gray, "out", grayShower,"in")
plasm.connect(sobelX, "out", abs1, "in")
plasm.connect(sobelY, "out", abs2, "in")
plasm.connect(abs1, "out", adder , "a")
plasm.connect(abs2, "out", adder , "b")
plasm.connect(adder, "out", sobelShower, "in")

#print plasm.viz()
ecto.view_plasm(plasm)

while(imshow.outputs.out.get() != 27):
    plasm.mark_dirty(video)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(sobelShower)
    plasm.go(grayShower)
    plasm.go(imshow)




