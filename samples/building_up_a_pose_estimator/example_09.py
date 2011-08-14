#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc
from example_08 import PoseFromFiducial
#lil bit of ros
PKG = 'ecto_ros' # this package name
# import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
import sys
if "__main__" == __name__:
  ecto_ros.init(sys.argv,"pose_estimator")
  
  plasm = ecto.Plasm()
  sched = ecto.schedulers.Singlethreaded(plasm)

  #lil bit of debug On/Off
  debug = True
  if 'R' in sys.argv:
      debug = False

  #add our black box to the plasm.
  pose_from_fiducial = PoseFromFiducial(plasm,
                                        rows=5, cols=3, 
                                        pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                        imshow_name="feh",
                                        square_size=0.04, debug=debug)

  circle_drawer = calib.PatternDrawer('Circle Draw',
                                      rows=7, cols=3)


  sub_rgb = ecto_sensor_msgs.Subscriber_Image('Image sub',topic_name='image')
  im2mat_rgb = ecto_ros.Image2Mat('Image -> cv::Mat')
  rgb2bgr = imgproc.cvtColor('rgb -> bgr')
  pose_gen = ecto_ros.RT2PoseStamped('R,T -> PoseStamped',frame_id='/openni_rgb_optical_frame')
  pose_pub = ecto_geometry_msgs.Publisher_PoseStamped('Pose Pub',topic_name='dot_pose')
  plasm.connect(sub_rgb["output"] >> im2mat_rgb["image"],
                im2mat_rgb["image"] >> rgb2bgr[:],
                rgb2bgr[:] >> pose_from_fiducial['image'],
                pose_from_fiducial["R","T"] >> pose_gen["R","T"],
                pose_gen['pose'] >> pose_pub[:]
                )
                
  ecto.view_plasm(plasm)
  sched.execute()
