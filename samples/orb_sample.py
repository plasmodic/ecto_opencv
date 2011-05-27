#!/bin/python
import ecto
from ecto_opencv import imgproc, highgui, features2d
import time
#import orb as imgproc

debug = True
  
plasm = ecto.Plasm()

video = highgui.VideoCapture(video_device=0)
orb_m = features2d.ORB()
draw_kpts = features2d.DrawKeypoints()
imshow = highgui.imshow( name="ORB", waitKey=2, autoSize=True)
rgb2gray = imgproc.cvtColor (flag=7)

plasm.connect(video, "out", rgb2gray , "input")
plasm.connect(rgb2gray, "out", orb_m, "image")
plasm.connect(orb_m,"kpts",draw_kpts,"kpts")
plasm.connect(video, "out",draw_kpts,"input")
plasm.connect(draw_kpts, "output", imshow,"input")

print plasm.viz()
ecto.view_plasm(plasm)

prev = time.time()
count = 0

while(imshow.outputs.out != 27):
    plasm.execute()
    now = time.time()
    if(count == 200):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

