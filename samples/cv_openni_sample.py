#!/bin/python
import ecto
from ecto_opencv import highgui,cv_bp as opencv
import time
#import orb as imgproc

debug = True
  
plasm = ecto.Plasm()

capture = highgui.OpenNICapture(video_mode=opencv.CV_CAP_OPENNI_SXGA_15HZ)
image_view = highgui.imshow(name="RGB", waitKey= 10, autoSize=True)
depth_view = highgui.imshow(name="Depth", waitKey= 10, autoSize=True);

plasm.connect(capture, "image", image_view , "input")
plasm.connect(capture, "depth", depth_view , "input")

if debug:
  print plasm.viz()
  ecto.view_plasm(plasm)

prev = time.time()
count = 0

while(image_view.outputs.out not in (27,ord('q'))):
    plasm.execute()
    now = time.time()
    if(count == 200):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    

