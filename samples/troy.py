#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc, projector, features2d
#lil bit of ros
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
import sys
ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

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
        self.trans = imgproc.Translate("trans", x=0.04 * 3)

        self.debug = debug
        if self.debug:
            self.fps = highgui.FPSDrawer()
            self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                                     rows=rows, cols=cols)
            self.circle_display = highgui.imshow('Pattern show',
                                                 name='Pattern', 
                                                 waitKey=2, autoSize=True)
            self.pose_draw = calib.PoseDrawer('Pose Draw')

    def expose_outputs(self):
        return {
                'R': self.pose_calc['R'],
                'T': self.pose_calc['T'],
                'K': self.camera_info['K'],
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
                       self.circle_detector['out', 'found'] >> self.circle_drawer['points', 'found'],
                       self.camera_info['K'] >> self.pose_draw['K'],
                     ]
        return graph


if "__main__" == __name__:
  ecto_ros.init(sys.argv,"pose_estimator")
  
  plasm = ecto.Plasm()
  sched = ecto.schedulers.Singlethreaded(plasm)

  #lil bit of debug On/Off
  debug = True
  if 'R' in sys.argv:
      debug = False

  # define the input
  subs = dict(image=ImageSub(topic_name='camera/rgb/image_color', queue_size=0),
              depth=ImageSub(topic_name='camera/depth/image', queue_size=0),
              #depth_info=CameraInfoSub(topic_name='camera/depth/camera_info', queue_size=0),
              #image_info=CameraInfoSub(topic_name='camera/rgb/camera_info', queue_size=0),
              )

  sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)

  #add our black box to the plasm.
  pose_from_fiducial = PoseFromFiducial(plasm,
                                        rows=5, cols=3, 
                                        pattern_type="acircles",
                                        square_size=0.04, debug=debug)

  circle_drawer = calib.PatternDrawer('Circle Draw',
                                      rows=7, cols=3)

  udim = 0.04

  ppcm = 10
  xcm = 25
  ycm = 25
  srr = calib.SubrectRectifier("extractor",
                               xoffset=udim*6,
                               xsize_world=xcm*0.01, ysize_world=ycm*0.01,
                               xsize_pixels=xcm*ppcm, ysize_pixels=ycm*ppcm)

  # sub_rgb = ecto_sensor_msgs.Subscriber_Image('Image sub',topic_name='image')
  im2mat_rgb = ecto_ros.Image2Mat('Image -> cv::Mat')
  im2mat_depth = ecto_ros.Image2Mat('Depth -> cv::Mat')

  rgb2bgr = imgproc.cvtColor('rgb -> bgr')
  #pose_gen = ecto_ros.RT2PoseStamped('R,T -> PoseStamped',frame_id='/openni_rgb_optical_frame')
  #pose_pub = ecto_geometry_msgs.Publisher_PoseStamped('Pose Pub',topic_name='dot_pose')

  showy = highgui.imshow('Showy',
                         name='Pattern showy', 
                         waitKey=2, autoSize=True)

  depth_show = highgui.imshow('Depth',
                              name='Depth View', 
                              waitKey=2, autoSize=True)

  pose_from_plane = projector.PlaneFitter()
  
  dewarp_table = calib.SubrectRectifier("tabledewarper",
                                        xoffset=-0.3, yoffset=-0.3,
                                        xsize_world=0.6, ysize_world=0.6,
                                        xsize_pixels=int(ppcm*60), ysize_pixels=int(ppcm*60))

  table_showy = highgui.imshow('Dewarped Table',
                               name='Dewarped Table',
                               waitKey=2, autoSize=True)

  orb = features2d.ORB(n_levels=1)
  fast = features2d.FAST(thresh=5)
  orb_test = features2d.ORB(n_levels=1)
  fast_test = features2d.FAST(thresh=5)

  draw_kpts = features2d.DrawKeypoints()
  draw_kpts_test = features2d.DrawKeypoints()
  
  matcher = features2d.Matcher()
  hfitter = features2d.MatchRefinement()
  
  match_draw = features2d.DrawMatches()
  match_view = highgui.imshow("Matches",name="Matches",waitKey=-1, autoSize=True)

  # mainplane_draw = calib.PoseDrawer("main plane pose draw")

  #im2mat_depth["image"] >> pose_from_plane['depth'],
  #camera_info['K'] >> pose_from_plane['K'],
  #pose_from_plane['R', 'T'] >> warper['R', 'T'],
  #im2mat_rgb["image"] >> (pose_draw['image'],),
 
  
  plasm.connect(sync['image'] >> im2mat_rgb['image'],
                sync['depth'] >> im2mat_depth['image'],
                im2mat_depth['image'] >> depth_show[:],
                im2mat_depth['image'] >> pose_from_plane['depth'],
                pose_from_fiducial['K'] >> pose_from_plane['K'],
                im2mat_rgb['image'] >> rgb2bgr[:],
                rgb2bgr[:] >> (pose_from_fiducial['image'], srr['image'], dewarp_table['image']),
                pose_from_fiducial['R', 'T', 'K'] >> srr['R', 'T', 'K'],
                pose_from_plane['R', 'T'] >> dewarp_table['R', 'T'],
                pose_from_fiducial['K'] >> dewarp_table['K'],
                dewarp_table[:] >> (fast['image'],draw_kpts['input'],orb['image']),
                fast['kpts'] >> (draw_kpts['kpts'],orb['kpts']),
                draw_kpts['output'] >> showy[:],
                srr[:] >> (fast_test['image'],draw_kpts_test['input'],orb_test['image']),
                fast_test['kpts'] >> (draw_kpts_test['kpts'],orb_test['kpts']),
                draw_kpts_test['output'] >> table_showy[:],
                orb['descriptors'] >> matcher['train'],
                orb_test['descriptors'] >> matcher['test'],
                orb['kpts'] >> (hfitter['train'],match_draw['train']),
                orb_test['kpts'] >> (hfitter['test'],match_draw['test']),
                matcher['matches'] >> hfitter['matches'],
                hfitter['matches'] >> match_draw['matches'],
                srr[:] >> match_draw['test_image'],
                dewarp_table[:] >> match_draw['train_image'],
                match_draw[:] >> match_view[:],
                )
  ecto.view_plasm(plasm)
  #sched.execute(niter=0,nthreads=1)
  sched.execute()
  