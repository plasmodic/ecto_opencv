#!/bin/python
import ecto
from ecto_opencv import pcl
import time
#import orb as imgproc

debug = True

ecto.list_ecto_module(pcl)
  
plasm = ecto.Plasm()

grabber = pcl.KinectGrabber()
voxel = pcl.VoxelGrid(leaf_size=0.01)
viewer = pcl.CloudViewer()


plasm.connect(grabber, "output", voxel , "input")
plasm.connect(voxel, "output", viewer , "input")

if debug:
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
    

