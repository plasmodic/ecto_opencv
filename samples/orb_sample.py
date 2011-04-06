#!/bin/python
import ecto
from ecto.doc import print_module_doc, graphviz
from ecto_opencv import imgproc, highgui,orb
import time
#import orb as imgproc

debug = False

def hookUpORB(plasm, image, image_key, imshow):
    FAST = ecto.make(orb.FAST, thresh=45, N_max=5000)
    Harris = ecto.make(orb.Harris, N_max=300)
    plasm.connect(image, image_key, FAST, "image")
    plasm.connect(FAST, "out", Harris, "kpts")
    plasm.connect(image, image_key, Harris, "image")
    if(debug):
        draw_kpts = ecto.make(orb.DrawKeypoints)
        plasm.connect(Harris, "out", draw_kpts, "kpts")
        plasm.connect(image, image_key, draw_kpts, "image")
        plasm.connect(draw_kpts, "image", imshow, "in")
    return FAST
  
plasm = ecto.Plasm()

levels = 3
pyramid = ecto.make(orb.Pyramid, levels=levels, magnification=0, scale_factor=1.3)

#rescale needs the same number of levels as the pyramid
rescale = ecto.make(orb.PyramidRescale, levels=levels)
imshow = ecto.make(highgui.imshow)
#configure the imshow
ecto.config(imshow, name="video", waitKey=2, autoSize=True)
rgb2gray = ecto.make(imgproc.cvtColor, flag=7)

video = ecto.make(highgui.VideoCapture,video_device=0)

plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(rgb2gray, "out", pyramid, "in")

imshows = []
harrises = []
for i in pyramid.outputs:
    if("out" in i.key()):
        x = ecto.make(highgui.imshow, name=i.key(), waitKey=-1, autoSize=False)
        harrises.append(hookUpORB(plasm, pyramid, i.key(), x))
        imshows.append(x)
i = 0
for x in harrises:
    plasm.connect(pyramid, "scale_%d" % i, rescale, "scale_%d" % i)
    plasm.connect(x, "out", rescale, "kpts_%d" % i)
    i += 1

draw_kpts = ecto.make(orb.DrawKeypoints)
plasm.connect(rescale, "out", draw_kpts, "kpts")
plasm.connect(video, "out", draw_kpts, "image")
plasm.connect(draw_kpts, "image", imshow , "in")
graphviz(plasm)
ecto.view_plasm(plasm)

prev = time.time()
count = 0
    
while(imshow.o.out.get() != 27):
    plasm.markDirty(video)
    if debug:
        for x in imshows:
            plasm.go(x)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(imshow)
    now = time.time()
    if(count == 30):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

