#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, VideoCapture
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import PatternDetector, FiducialPoseFinder, \
     PatternDrawer, PoseDrawer, CameraIntrinsics, ASYMMETRIC_CIRCLES_GRID

rows = 5
cols = 3
square_size = 0.04 # in meters, 4 cm
calibration_file = 'camera.yml'
pattern_type = ASYMMETRIC_CIRCLES_GRID

pattern_show = imshow('Display', name='pattern')
rgb2gray = cvtColor('RGB -> Gray', flag=Conversion.RGB2GRAY)
video = VideoCapture(video_device=0)
circle_detector = PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type,
                                  square_size=square_size)
circle_drawer = PatternDrawer(rows=rows, cols=cols)
poser = FiducialPoseFinder()
pose_drawer = PoseDrawer()
camera_intrinsics = CameraIntrinsics(camera_file=calibration_file)

plasm = ecto.Plasm()
plasm.connect(video['image'] >> (rgb2gray['image'], circle_drawer['input']),
            rgb2gray['image'] >> circle_detector['input'],
            circle_detector['out', 'found'] >> circle_drawer['points', 'found'],
            camera_intrinsics['K'] >> poser['K'],
            circle_detector['out', 'ideal', 'found'] >> poser['points', 'ideal', 'found'],
            poser['R', 'T'] >> pose_drawer['R', 'T'],
            circle_drawer['out'] >> pose_drawer['image'],
            camera_intrinsics['K'] >> pose_drawer['K'],
            pose_drawer['output'] >> pattern_show['image'],
            )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Estimate the pose of a dot pattern.')

