#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow, ImageReader, VideoCapture, MatPrinter
from ecto_opencv.calib import PatternDetector, PatternDrawer,\
     FiducialPoseFinder, PoseDrawer, \
     CHESSBOARD, CameraIntrinsics
from ecto_opencv.imgproc import cvtColor, Conversion
import sys
import argparse


parser = argparse.ArgumentParser(description='Pose from a list of files.')
parser.add_argument('--images', nargs='+', help='A list of images.')
parser.add_argument('--camera', help='The camera file.')
args = parser.parse_args()

image_reader = ImageReader(file_list=ecto.list_of_strings(args.images))

rows = 4
cols = 5
square_size = 0.025 #2.5 cm
pattern_type = CHESSBOARD

rgb2gray = cvtColor(flag=Conversion.RGB2GRAY)

detector = PatternDetector(rows=rows, cols=cols,
                           pattern_type=pattern_type,
                           square_size=square_size)

pattern_drawer = PatternDrawer(rows=rows, cols=cols)
camera_intrinsics = CameraIntrinsics(camera_file=args.camera)
poser = FiducialPoseFinder()
pose_drawer = PoseDrawer()
plasm = ecto.Plasm()
plasm.connect(image_reader['image'] >> (pattern_drawer['input'],
                                        rgb2gray['image']),
              rgb2gray['image'] >> detector['input'],
              
              detector[ 'ideal', 'out', 'found'] >> poser['ideal', 'points', 'found'],
              camera_intrinsics['K'] >> poser['K'],
              detector['out', 'found'] >> pattern_drawer['points', 'found'],
              poser['R', 'T'] >> pose_drawer['R', 'T'],
              poser['R'] >> MatPrinter(name='R')['mat'],
              poser['T'] >> MatPrinter(name='T')['mat'],
              
              pattern_drawer['out'] >> pose_drawer['image'],
              camera_intrinsics['K'] >> pose_drawer['K'],
              pose_drawer['output'] >> imshow(name='Pose', waitKey=0)['image'],
              )

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()