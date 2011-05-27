#!/bin/python
import ecto
from ecto_opencv import pcl
import time
#import orb as imgproc

debug = True
  
plasm = ecto.Plasm()

grabber = pcl.KinectGrabber()
viewer = pcl.CloudViewer()

plasm.connect(grabber, "output", viewer , "input")

print plasm.viz()
ecto.view_plasm(plasm)

prev = time.time()
count = 0

while not viewer.outputs.stop:
    plasm.execute()
    now = time.time()
    if(count == 200):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

