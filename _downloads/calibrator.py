#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, VideoCapture
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID
from ecto_opencv.imgproc import cvtColor, Conversion
import sys
rows = 5
cols = 3
square_size = 0.04 #4 cm
pattern_type = ASYMMETRIC_CIRCLES_GRID
n_obs = 50
calibration_file = 'camera_new.yml'

video = VideoCapture(video_device=0)
rgb2gray = cvtColor(flag=Conversion.RGB2GRAY)
circle_detector = PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type, square_size=square_size)
camera_calibrator = CameraCalibrator(output_file_name=calibration_file, n_obs=n_obs)
circle_drawer = PatternDrawer(rows=rows, cols=cols)

plasm = ecto.Plasm()
plasm.connect(video['image'] >> (circle_drawer['input'], camera_calibrator['image'], rgb2gray['image']),
              rgb2gray['image'] >> circle_detector['input'],
              circle_drawer['out'] >> imshow(name='pattern')['image'],
              circle_detector[ 'ideal', 'out', 'found'] >> camera_calibrator['ideal', 'points', 'found'],
              circle_detector['out', 'found'] >> circle_drawer['points', 'found'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Calibrate a camera using a dot pattern.')
