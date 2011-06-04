#!/bin/python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc,tod
import time
#import orb as imgproc

debug = True

class PoseEstimator:
    
    def __init__(self):
        self.draw_debug = debug
        self.circle_drawer = calib.PatternDrawer(rows=7, cols=3)
        self.pattern_show = highgui.imshow(name="pattern", waitKey=-1, autoSize=True)
        self.pose_drawer = calib.PoseDrawer()            
        self.rgb2gray = imgproc.cvtColor(flag=7)
        self.circle_detector = calib.PatternDetector(rows=7, cols=3, pattern_type="acircles", square_size=0.03)
        self.poser = calib.FiducialPoseFinder()
        self.camera_intrinsics = calib.CameraIntrinsics(camera_file="camera.kinect.vga.yml")
        
    def declare_graph(self, plasm, image_source):
        plasm.connect(image_source, "image", self.rgb2gray, "input")
        plasm.connect(self.rgb2gray, "out", self.circle_detector, "input")
        plasm.connect(self.camera_intrinsics, "K", self.poser, "K")
        plasm.connect(self.circle_detector, "out", self.poser, "points")
        plasm.connect(self.circle_detector, "ideal", self.poser, "ideal")
        plasm.connect(self.circle_detector, "found", self.poser, "found")
        if self.draw_debug:
            plasm.connect(image_source, "image", self.circle_drawer, "input")
            plasm.connect(self.circle_detector, "out", self.circle_drawer, "points")
            plasm.connect(self.circle_detector, "found", self.circle_drawer, "found")
        plasm.connect(self.poser, "R", self.pose_drawer, "R")
        plasm.connect(self.poser, "T", self.pose_drawer, "T")
        plasm.connect(self.circle_drawer, "out", self.pose_drawer, "image")
        plasm.connect(self.camera_intrinsics, "K", self.pose_drawer, "K")
        plasm.connect(self.pose_drawer, "output", self.pattern_show, "input")
            
        
plasm = ecto.Plasm()

pose_est = PoseEstimator()

capture = highgui.OpenNICapture(video_mode=opencv.CV_CAP_OPENNI_VGA_30HZ)
image_view = highgui.imshow(name="RGB", waitKey=10, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey=-1, autoSize=True)

masker = tod.PlanarSegmentation()
if debug:
    depth_view = highgui.imshow(name="Depth", waitKey=-1, autoSize=True);
    plasm.connect(capture, "image", image_view , "input")
    plasm.connect(capture, "depth", depth_view , "input")
    
pose_est.declare_graph(plasm, capture)

plasm.connect(pose_est.poser, "R",masker,"R")
plasm.connect(pose_est.poser, "T",masker,"T")
plasm.connect(pose_est.camera_intrinsics, "K",masker,"K")
plasm.connect(capture, "depth", masker , "depth")
plasm.connect(masker,"mask", mask_view,"input")


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
