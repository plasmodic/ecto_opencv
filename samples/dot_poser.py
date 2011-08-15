#!/usr/bin/env python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc

debug = True

rows = 7
cols = 3
square_size = 0.03 # in meters

pattern_show = highgui.imshow('Display', name="pattern", waitKey=10, autoSize=True)
rgb2gray = imgproc.cvtColor('RGB -> Gray', flag=imgproc.Conversion.RGB2GRAY)
video = highgui.VideoCapture(video_device=0)
circle_detector = calib.PatternDetector(rows=rows, cols=cols,
                                        pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                        square_size=square_size)
circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
poser = calib.FiducialPoseFinder()
pose_drawer = calib.PoseDrawer()
camera_intrinsics = calib.CameraIntrinsics(camera_file="camera.yml")

plasm = ecto.Plasm()
plasm.connect(video["image"] >> (rgb2gray["input"], circle_drawer['input']),
            rgb2gray["out"] >> circle_detector["input"],
            circle_detector["out", "found"] >> circle_drawer["points", "found"],
            camera_intrinsics["K"] >> poser["K"],
            circle_detector["out", "ideal", "found"] >> poser["points", "ideal", "found"],
            poser["R", "T"] >> pose_drawer["R", "T"],
            circle_drawer["out"] >> pose_drawer["image"],
            camera_intrinsics["K"] >> pose_drawer["K"],
            pose_drawer["output"] >> pattern_show["input"],
            )

if __name__ == '__main__':
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(8)

