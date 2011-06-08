#!/bin/python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, imgproc
import time
#import orb as imgproc

debug = True
  
plasm = ecto.Plasm()

capture = highgui.OpenNICapture(video_mode=opencv.CV_CAP_OPENNI_VGA_30HZ)
image_view = highgui.imshow(name="RGB", waitKey=10, autoSize=True)
depth_view = highgui.imshow(name="Depth", waitKey=10, autoSize=True);
sobel_view = highgui.imshow(name="Sobeled Depth", waitKey=10, autoSize=True);
sobelX = imgproc.Scharr(x=1, y=0)
sobelY = imgproc.Scharr(x=0, y=1)
gaussian = imgproc.GaussianBlur(sigma=2.0)
kmeans = imgproc.CartToPolar()

plasm.connect(capture,"depth",gaussian,"input")
plasm.connect(capture, "image", image_view , "input")
plasm.connect(gaussian, "out", depth_view , "input")
plasm.connect(gaussian, "out", sobelY , "input")
plasm.connect(gaussian, "out", sobelX, "input")
plasm.connect(sobelX, "out", kmeans, "x")
plasm.connect(sobelY, "out", kmeans, "y")
plasm.connect(kmeans, "angle", sobel_view, "input")


if debug:
  print plasm.viz()
  ecto.view_plasm(plasm)

prev = time.time()
count = 0

while(image_view.outputs.out not in (27, ord('q'))):
    plasm.execute()
    now = time.time()
    if(count == 200):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
