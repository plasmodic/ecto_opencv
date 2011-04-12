import ecto_opencv.opencv as opencv
import sys
import time
vid = opencv.VideoCapture()
vid.open(opencv.CV_CAP_OPENNI)

if(not vid.isOpened()):
    print "Could not open video device!"
    sys.exit()

depth = opencv.Mat()
rgb = opencv.Mat()
prev = time.time()
count = 0
out = opencv.buffer()
show = opencv.Mat()
show2 = opencv.Mat()
points = opencv.Mat()
while(True):
    if(not vid.grab()):
        print "bad grab"
        continue
    if(not vid.retrieve(depth,opencv.CV_CAP_OPENNI_DEPTH_MAP)):
        print "bad depth"
        continue
    if(not vid.retrieve(rgb,opencv.CV_CAP_OPENNI_GRAY_IMAGE)):
        print "bad rgb"
        continue

    depth.convertTo(show, opencv.CV_8U,0.05,0);
    opencv.imshow("depth", show )
    opencv.imshow("rgb",rgb)
    opencv.imwrite("depth.jpg",show)
    show2 = opencv.imread("depth.jpg")
    opencv.depth2points(depth,points)
    #opencv.savepoints("points.ply",points)
    opencv.imshow("points",points)
    #opencv.imshow("depth jpg",show2)
    if(opencv.waitKey(3) & 0xff == ord('q')):
        break;
    now = time.time()
    if(count == 30):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1
    
