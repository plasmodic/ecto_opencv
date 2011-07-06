#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc

class PoseFromFiducial(ecto.BlackBox):
    def __init__(self, plasm):
        ecto.BlackBox.__init__(self, plasm)
        self.video_cap = highgui.VideoCapture(video_device=0)
        self.fps = highgui.FPSDrawer()
        self.rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)
        self.circle_detector = calib.PatternDetector('Dot Detector',
                                                rows=7, cols=3, pattern_type="acircles",
                                                square_size=0.03)
        self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                            rows=7, cols=3)
        self.circle_display = highgui.imshow('Pattern show',
                                        name='Pattern', waitKey=10, maximize=True)
        self.pose_calc = calib.FiducialPoseFinder('Pose Calc')
        self.pose_draw = calib.PoseDrawer('Pose Draw')
        self.camera_info = calib.CameraIntrinsics('Camera Info', camera_file="camera.yml")

    def expose_outputs(self):
        return {
               }
    def expose_parameters(self):
        return {
                }
    def connections(self):
        return [
                self.video_cap['image'] >> self.circle_drawer['input'],
                self.circle_drawer['out'] >> self.pose_draw['image'],
                self.pose_draw['output'] >> self.fps['image'],
                self.fps['image'] >> self.circle_display['input'],
                self.video_cap['image'] >> self.rgb2gray['input'],
                self.rgb2gray['out'] >> self.circle_detector['input'],
                self.circle_detector['out', 'found'] >> self.circle_drawer['points', 'found'],
                self.camera_info['K'] >> (self.pose_calc['K'], self.pose_draw['K']),
                self.circle_detector['out', 'ideal', 'found'] >> self.pose_calc['points', 'ideal', 'found'],
                self.pose_calc['R', 'T'] >> self.pose_draw['R', 'T']
               ]

plasm = ecto.Plasm()
sched = ecto.schedulers.Singlethreaded(plasm)

#add our black box to the plasm.
pose_from_fiducial = PoseFromFiducial(plasm)
pose_from_fiducial.connect()
ecto.view_plasm(plasm)
sched.execute()
