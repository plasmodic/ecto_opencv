#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc

plasm = ecto.Plasm()
sched = ecto.schedulers.Threadpool(plasm)

video_cap = highgui.VideoCapture(video_device=0)
fps = highgui.FPSDrawer()
rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)

display_strand = ecto.Strand()

checker_detector = calib.PatternDetector('Checker Detector',
                                         rows=5, cols=4,
                                         pattern_type="chessboard",
                                         square_size=0.03)
circle_detector = calib.PatternDetector('Dot Detector',
                                        rows=7, cols=3, pattern_type="acircles",
                                        square_size=0.03)
circle_drawer = calib.PatternDrawer('Circle Draw',
                                    rows=7, cols=3)
checker_drawer = calib.PatternDrawer('Checker Draw',
                                     rows=5, cols=4)
circle_display = highgui.imshow('Pattern show',
                                name='Pattern', waitKey=10, autoSize=True,
                                strand=display_strand)

pose_calc = calib.FiducialPoseFinder('Pose Calc')
pose_draw = calib.PoseDrawer('Pose Draw')
camera_info = calib.CameraIntrinsics('Camera Info', camera_file="camera.yml")

plasm.connect( video_cap['image'] >> circle_drawer['input'],
               circle_drawer['out'] >> checker_drawer['input'],
               checker_drawer['out'] >> pose_draw['image'],
               pose_draw['out'] >> fps['image'],
               fps['image'] >> circle_display['input'],
               video_cap['image'] >> rgb2gray['input'],
               rgb2gray['out'] >> (circle_detector['input'], checker_detector['input']),
               circle_detector['out', 'found'] >> circle_drawer['points', 'found'],
               checker_detector['out', 'found'] >> checker_drawer['points', 'found'],
               camera_info['K'] >> (pose_calc['K'], pose_draw['K']),
               circle_detector['out', 'ideal', 'found'] >> pose_calc['points', 'ideal', 'found'],
               pose_calc['R', 'T'] >> pose_draw['R', 'T']
              )

ecto.view_plasm(plasm)
sched.execute()
