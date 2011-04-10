#!/bin/python
import ecto
from ecto.doc import print_module_doc, graphviz
from ecto_opencv import imgproc, highgui,orb
import time
#import orb as imgproc

debug = True
score_zippers = []
def hookUpORB(plasm, image, image_key, imshow):
    FAST = ecto.make(orb.FAST, thresh=100, N_max=50000)
    Harris = ecto.make(orb.Harris, N_max=50000)

    plasm.connect(image, image_key, FAST, "image")
    plasm.connect(FAST, "out", Harris, "kpts")
    plasm.connect(image, image_key, Harris, "image")
    if(debug):
        draw_kpts = ecto.make(orb.DrawKeypoints)
        plasm.connect(Harris, "out", draw_kpts, "kpts")
        plasm.connect(image, image_key, draw_kpts, "image")
        plasm.connect(draw_kpts, "image", imshow, "in")
        score_zipper = ecto.make(orb.ScoreZipper)
        plasm.connect(FAST, "out", score_zipper, "kpts_0")
        plasm.connect(Harris, "out", score_zipper, "kpts_1")
        score_zippers.append(score_zipper)
    return Harris
  
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

scores = open("scores.txt",'w')
while(imshow.o.out.get() != 27):
    plasm.mark_dirty(video)
    if debug:
        for x in imshows:
            plasm.go(x)
        for x in score_zippers:
            plasm.go(x)
            for s in x.o.scores.val:
              scores.write("%f %f\n"%(s.first,s.second))
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(imshow)
    now = time.time()
    if(count == 200):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

