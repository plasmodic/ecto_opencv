#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageReader, VideoCapture
from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, CHESSBOARD
from ecto_opencv.imgproc import cvtColor, Conversion
import sys
import argparse


parser = argparse.ArgumentParser(description='Calibrate from a directory of files.')
parser.add_argument('--images', nargs='+', help='A list of images.')
parser.add_argument('--output', help='The output file.')
args = parser.parse_args()

image_reader = ImageReader(file_list=ecto.list_of_strings(args.images))

rows = 5
cols = 4
square_size = 0.025 #2.5 cm
pattern_type = CHESSBOARD


calibration_file = args.output

rgb2gray = cvtColor(flag=Conversion.RGB2GRAY)

detector = PatternDetector(rows=rows, cols=cols,
                           pattern_type=pattern_type,
                           square_size=square_size)

camera_calibrator = CameraCalibrator(output_file_name=calibration_file,
                                     n_obs=len(args.images) -1,
                                     quit_when_calibrated=False)

pattern_drawer = PatternDrawer(rows=rows, cols=cols)

plasm = ecto.Plasm()
plasm.connect(image_reader['image'] >> (pattern_drawer['input'],
                                        camera_calibrator['image'],
                                        rgb2gray['image']),
              rgb2gray['image'] >> detector['input'],
              
              detector[ 'ideal', 'out', 'found'] >> camera_calibrator['ideal', 'points', 'found'],
              detector['out', 'found'] >> pattern_drawer['points', 'found'],
              pattern_drawer['out'] >> imshow(name='pattern')['image'],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()