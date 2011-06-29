#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc

debug = True

plasm = ecto.Plasm()
rows = 5
cols = 3
square_size = 0.03 # in millis

pattern_show = highgui.imshow(name="pattern", waitKey=10, autoSize=True)
rgb2gray = imgproc.cvtColor(flag=7)
video = highgui.VideoCapture(video_device=0)
circle_detector = calib.PatternDetector(rows=rows, cols=cols, pattern_type="acircles", square_size=square_size)
circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
poser = calib.FiducialPoseFinder()
pose_drawer = calib.PoseDrawer()
camera_intrinsics = calib.CameraIntrinsics(camera_file="camera.yml")
splitter = imgproc.ChannelSplitter()

plasm.connect(video["image"] >> (rgb2gray["input"], splitter["input"]),
              splitter["out_0"] >> (circle_detector["input"], circle_drawer["input"])
                                    )
                                    
plasm.connect(circle_detector['out'] >> circle_drawer['points'],
              circle_detector['found'] >> circle_drawer['found'],
              camera_intrinsics['K'] >> poser['K'],
              circle_detector['out'] >> poser['points'],
              circle_detector['ideal'] >> poser['ideal'],
              circle_detector['found'] >> poser['found'],
              poser['R'] >> pose_drawer['R'],
              poser['T'] >> pose_drawer['T'],
              circle_drawer['out'] >> pose_drawer['image'],
              camera_intrinsics['K'] >> pose_drawer['K'],
              pose_drawer['output'] >> pattern_show['input'],
              )


if debug:
    ecto.view_plasm(plasm)

sched = ecto.schedulers.Threadpool(plasm)

sched.execute(nthreads=8)

#while(pattern_show.outputs.out != 27):
#    plasm.execute()
    

