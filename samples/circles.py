#!/bin/python
import ecto
from ecto_opencv import imgproc, highgui, features2d, calib
import time
#import orb as imgproc

debug = False
#debug = True

plasm = ecto.Plasm()

video = highgui.VideoCapture(video_device=0)
rgb2gray = imgproc.cvtColor (flag=7)
gaussian = imgproc.GaussianBlur(sigma=2.0)
circle_drawer = calib.CircleDrawer()
pong = calib.PingPongDetector(dp=2,maxRadius=100, minRadius=10,param1=180,param2=90,minDist=20)
print pong.__doc__
show_circles = highgui.imshow("show circles", name="Circles", waitKey=10)
plasm.connect(
              video["image"] >> (rgb2gray["input"], circle_drawer["image"]),
              rgb2gray["out"] >> gaussian["input"],
              gaussian["out"] >> pong["image"],
              pong["circles"] >> circle_drawer["circles"],
              circle_drawer["image"] >> show_circles["input"]
              )
if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

prev = time.time()
count = 0

while(show_circles.outputs.out not in (ord('q'), 27)):
    plasm.execute(1)
    now = time.time()
    if(count == 200):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

