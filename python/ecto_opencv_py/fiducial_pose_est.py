#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc

class PoseFromFiducial(ecto.BlackBox):
    def __init__(self, plasm, rows, cols, pattern_type, square_size, offset_x=0, offset_y=0, offset_z=0, debug=True):
        ecto.BlackBox.__init__(self, plasm)
        self.video_cap = ecto.Passthrough('Image Input')
        self.camera_info = ecto.Passthrough('K')

        self.circle_detector = calib.PatternDetector('Dot Detector',
                                                rows=rows, cols=cols,
                                                pattern_type=pattern_type,
                                                square_size=square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                offset_z=offset_z)
        self.pose_calc = calib.FiducialPoseFinder('Pose Calc')
        self.debug = debug
        self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                                     rows=rows, cols=cols)
        self.pose_draw = calib.PoseDrawer('Pose Draw')
    def expose_outputs(self):
        return {
                'R': self.pose_calc['R'],
                'T': self.pose_calc['T'],
                'found':  self.circle_detector['found'],
                'points': self.circle_detector['out'],
                'ideal': self.circle_detector['out'],
                'debug_image':  self.pose_draw['output'],
               }
    def expose_inputs(self):
        return {
                'image': self.video_cap[:],
                'color_image': self.circle_drawer['input'],
                'K': self.camera_info[:]
               }
    def expose_parameters(self):
        return {
                }
    def connections(self):
        graph = [self.video_cap[:] >> self.circle_detector['input'],
                 self.camera_info[:] >> self.pose_calc['K'],
                 self.circle_detector['out', 'ideal', 'found'] >> self.pose_calc['points', 'ideal', 'found'],
                 self.circle_drawer['out'] >> self.pose_draw['image'],
                 self.pose_calc['R', 'T'] >> self.pose_draw['R', 'T'],
                 self.circle_detector['out', 'found'] >> self.circle_drawer['points', 'found'],
                 self.camera_info[:] >> self.pose_draw['K'],
               ]
        return graph
