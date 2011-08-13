#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, calib, imgproc, projector
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
from ecto_opencv import highgui
import sys

ImageSub = ecto_sensor_msgs.Subscriber_Image
PointCloud2Sub = ecto_sensor_msgs.Subscriber_PointCloud2
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

DEBUG = True

# define the input
subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
            #image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
            depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
            #depth_info=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0
            points=PointCloud2Sub(topic_name='/camera/depth_registered/points', queue_size=0)
         )

sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)

im2mat_rgb = ecto_ros.Image2Mat()
im2mat_depth = ecto_ros.Image2Mat()
bgr2gray = imgproc.cvtColor('bgr -> gray', flag=imgproc.CV_BGR2GRAY)
camera_info = calib.CameraIntrinsics('Camera Info', camera_file="camera.yml")

calibrator = projector.Calibrator()
plasm = ecto.Plasm()
offset_x = 0

main_display = highgui.imshow("rgb show", name="rgb", waitKey=5,)
graph = [
            sync["image"] >> im2mat_rgb["image"],
            sync["depth"] >> im2mat_depth["image"],
            im2mat_rgb["image"] >> bgr2gray["input"],
            bgr2gray["out"] >> main_display["input"]
        ]

print "Press 's' to capture a view."

# add the display of the pattern
video_display = highgui.imshow('pattern',
                                name='video_cap', waitKey=2, maximize=False, autoSize=True)
# Deal with the warping
warper = projector.ImageWarper(projection_file='projector_calibration.yml', offset_x=0, offset_y=0, scale=1)
warper_kinect = projector.CameraWarper(projection_file='projector_calibration.yml')

pose_from_plane = ecto.If("throttled plane fitter", cell=projector.PlaneFitter())

truer = ecto.TrueEveryN(n=120)
button = projector.ButtonProjector(radius=int(.10 * 480) / 2, image_width=640, image_height=480)
depthTo3d = calib.DepthTo3d()

graph += [im2mat_depth["image"] >> pose_from_plane['depth'],
          truer['flag'] >> pose_from_plane['__test__'],
          camera_info['K'] >> pose_from_plane['K'],
          pose_from_plane['R', 'T'] >> warper['R', 'T'],
           button["button_image"]>> 
              (warper['image'],
              highgui.imshow("raw_buttons", name="raw_buttons", waitKey= -1)[:],
              ),
          button['mask'] >> warper_kinect['image'],
          camera_info['K'] >> warper_kinect['K'],
          warper_kinect['output'] >> highgui.imshow("warped mask", name="warped mask", waitKey= -1,)[:],
          warper['output'] >> highgui.imshow("warped image", name="warped", waitKey= -1,)[:],
          ]

graph += [
            camera_info['K'] >> depthTo3d['K'],
            im2mat_depth["image"] >> depthTo3d['depth']
          ]

ibox_0 = projector.InteractionBox(box_width=0.1,
                                  box_height=0.1,
                                  box_depth=0.1,
                                  origin_x=0,
                                  origin_y=0,
                                  origin_z=0
                                  )
graph += [
            depthTo3d[:] >> ibox_0["points3d"],
            pose_from_plane['R', 'T'] >> ibox_0['R', 'T'],
          ]

#pose publishing    
pose_gen = ecto_ros.RT2PoseStamped('R,T -> PoseStamped', frame_id='/camera_rgb_optical_frame')
pose_pub = ecto_geometry_msgs.Publisher_PoseStamped('Pose Pub', topic_name='dot_pose')
graph += [  
            pose_from_plane["R", "T"] >> pose_gen["R", "T"],
            pose_gen['pose'] >> pose_pub[:]
        ]

plasm.connect(graph)

# display DEBUG data if needed
if DEBUG:
#    print plasm.viz()
    ecto.view_plasm(plasm)

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "ecto_node")
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
