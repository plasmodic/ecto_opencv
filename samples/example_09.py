#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc
from example_08 import PoseFromFiducial
#lil bit of ros
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs
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
                                        rows=7, cols=3, 
                                        pattern_type="acircles",
                                        square_size=0.03, debug=debug)

  sub_rgb = ecto_sensor_msgs.Subscriber_Image("image_sub",topic_name='image')
  im2mat_rgb = ecto_ros.Image2Mat("Image -> cv::Mat")
  
  plasm.connect(
                sub_rgb["output"]>>im2mat_rgb["image"],
                im2mat_rgb["image"] >> pose_from_fiducial['image'],
                )
                
  ecto.view_plasm(plasm)
  sched.execute()
