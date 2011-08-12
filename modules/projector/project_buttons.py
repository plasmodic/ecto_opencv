#!/usr/bin/env python
PKG = 'ecto_projector'
import roslib; #roslib.load_manifest(PKG)
import ecto
from ecto_opencv import highgui, calib, imgproc, projector
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
from ecto_opencv import highgui
from optparse import OptionParser
import sys
from ecto_opencv_py.fiducial_pose_est import PoseFromFiducial

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

DEBUG = True

########################################################################################################################

def parse_options():
    parser = OptionParser()
    parser.add_option("-c", "--config_file", dest="config_file",
                      help="the file containing the configuration")
    parser.add_option("-a", "--bag", dest="bag", help="The bag to analyze")

    (options, args) = parser.parse_args()
    return options

########################################################################################################################

def do_projector():
    options = parse_options()

    # define the input
    subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                #image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                #depth_info=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0
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
    warper = projector.ImageWarper(projection_file='projector_calibration.yml', offset_x=0,offset_y=0,scale=1)
    
    pose_from_plane = ecto.If("throttled plane fitter",cell=projector.PlaneFitter())
    
    truer = ecto.TrueEveryN(n=120)

    graph += [im2mat_depth["image"] >> pose_from_plane['depth'],
              truer['flag'] >> pose_from_plane['__test__'],
              camera_info['K'] >> pose_from_plane['K'],
              pose_from_plane['R', 'T'] >> warper['R', 'T'],
              projector.ButtonProjector(radius=int(.10 * 480)/2,image_width=640,image_height=480)["button_image"] >>
                  (warper['image'],
                  highgui.imshow("raw_buttons",name="raw_buttons",waitKey=-1)[:],
                  ),
              warper['output'] >> highgui.imshow("warped image", name="warped", waitKey= -1,)[:],
              ]


    #pose publishing    
    pose_gen = ecto_ros.RT2PoseStamped('R,T -> PoseStamped',frame_id='/camera_rgb_optical_frame')
    pose_pub = ecto_geometry_msgs.Publisher_PoseStamped('Pose Pub',topic_name='dot_pose')
    graph += [  
                pose_from_plane["R","T"] >> pose_gen["R","T"],
                pose_gen['pose'] >> pose_pub[:]
            ]

    plasm.connect(graph)

    # display DEBUG data if needed
    if False:
    #    print plasm.viz()
        ecto.view_plasm(plasm)

    # execute the pipeline
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()


if __name__ == "__main__":
    ecto_ros.init(sys.argv, "ecto_node")
    do_projector()

