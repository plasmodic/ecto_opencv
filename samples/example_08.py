#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc

class PoseFromFiducial(ecto.BlackBox):
    def __init__(self, plasm,rows,cols,pattern_type,square_size,debug=True):
        ecto.BlackBox.__init__(self, plasm)
        if debug:
            print self.__class__, "enabling debug nodes"
        self.video_cap = ecto.Passthrough('Image Input')
        self.rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)
        self.circle_detector = calib.PatternDetector('Dot Detector',
                                                     rows=rows, cols=cols, 
                                                     pattern_type=pattern_type,
                                                     square_size=square_size)
        self.pose_calc = calib.FiducialPoseFinder('Pose Calc')
        self.camera_info = calib.CameraIntrinsics('Camera Info', 
                                                  camera_file="camera.yml")
        self.debug = debug
        if self.debug:
            self.fps = highgui.FPSDrawer()
            self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                                     rows=rows, cols=cols)
            self.circle_display = highgui.imshow(name='Pattern', 
                                                 waitKey=2, autoSize=True)
            self.pose_draw = calib.PoseDrawer('Pose Draw')

    def expose_outputs(self):
        return {
                'R': self.pose_calc['R'],
                'T': self.pose_calc['T'],
               }
    def expose_inputs(self):
        return {
                'image': self.video_cap[:],
               }
    def expose_parameters(self):
        return {
                }
    def connections(self):
        graph = [self.video_cap[:] >> self.rgb2gray['input'],
                 self.rgb2gray['out'] >> self.circle_detector['input'],
                 self.camera_info['K'] >> self.pose_calc['K'],
                 self.circle_detector['out', 'ideal', 'found'] >> 
                 self.pose_calc['points', 'ideal', 'found'],
               ]
        
        if self.debug:
            graph += [ self.video_cap[:] >> self.circle_drawer['input'],
                       self.circle_drawer['out'] >> self.pose_draw['image'],
                       self.pose_draw['output'] >> self.fps['image'],
                       self.fps['image'] >> self.circle_display['input'],
                       self.pose_calc['R', 'T'] >> self.pose_draw['R', 'T'],
                       self.circle_detector['out', 'found'] >> 
                       self.circle_drawer['points', 'found'],
                       self.camera_info['K'] >> self.pose_draw['K'],
                     ]
        return graph

if "__main__" == __name__:
  import sys
  
  plasm = ecto.Plasm()
  sched = ecto.schedulers.Threadpool(plasm)

  #lil bit of debug On/Off
  debug = True
  if 'R' in sys.argv:
      debug = False

  #add our black box to the plasm.
  pose_from_fiducial = PoseFromFiducial(plasm,
                                        rows=5, cols=3, 
                                        pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                        square_size=0.04, debug=debug)

  pff2 =  PoseFromFiducial(plasm, rows=5, cols=3, 
                           pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                           square_size=0.04, debug=debug)

  video_cap = highgui.VideoCapture(video_device=0)
  invert = imgproc.BitwiseNot()
  
  plasm.connect(video_cap['image'] >> (pose_from_fiducial['image'], invert['input']),
                invert[:] >> pff2['image']
                )
                
  ecto.view_plasm(plasm)
  sched.execute()

