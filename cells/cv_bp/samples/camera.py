import ecto_opencv.cv_bp as opencv

import sys

device = 0
if len(sys.argv) == 2:
  device = sys.argv[1]
  try:
    #attempt to turn into an int
    device = int(device)
  except:
    pass
    
capture = opencv.VideoCapture(device)

img = opencv.Mat()

print "press q or ESC to exit."

while True:
    capture.read(img)
    if img.empty(): continue
    opencv.imshow("camera",img)
    if opencv.waitKey(10) in ( ord('q'), 27 ):
        break
