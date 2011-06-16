#!/bin/python
import ecto
from ecto_opencv import imgproc, highgui, features2d
import time
#import orb as imgproc

debug = False
#debug = True

plasm = ecto.Plasm()

video = highgui.VideoCapture(video_device=0)
orb_m = features2d.ORB(n_features=2500)
draw_kpts = features2d.DrawKeypoints()
imshow = highgui.imshow(name="ORB", waitKey=2, autoSize=True)
rgb2gray = imgproc.cvtColor (flag=7)

plasm.connect(video, "image", rgb2gray , "input")
plasm.connect(rgb2gray, "out", orb_m, "image")
plasm.connect(orb_m, "kpts", draw_kpts, "kpts")
plasm.connect(video, "image", draw_kpts, "input")
plasm.connect(draw_kpts, "output", imshow, "input")

if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

prev = time.time()
count = 0

while(imshow.outputs.out != 27):
    plasm.execute(30)
    now = time.time()
    if(count == 200):
        print "%f fps" % (30 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

