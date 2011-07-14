#!/usr/bin/env python
PKG = 'ecto_projector'
import roslib; #roslib.load_manifest(PKG)
import ecto
from ecto_opencv import highgui, calib, imgproc, projector
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
from optparse import OptionParser
import sys

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
    subs = dict(image=ImageSub(topic_name='camera/rgb/image_color', queue_size=0),
                depth=ImageSub(topic_name='camera/depth/image', queue_size=0),
                depth_info=CameraInfoSub(topic_name='camera/depth/camera_info', queue_size=0),
                image_info=CameraInfoSub(topic_name='camera/rgb/camera_info', queue_size=0),
             )

    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)

    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    invert = imgproc.BitwiseNot()
    brg2rgb = imgproc.cvtColor('bgr -> rgb', flag=4)

    camera_info = calib.CameraIntrinsics('Camera Info', camera_file="camera.yml")

    pattern_draw = projector.PatternProjector()
    circle_detector = calib.PatternDetector('Dot Detector',
                                                rows=5, cols=3,
                                                pattern_type="acircles",
                                                square_size=0.04)
    circle_drawer = calib.PatternDrawer('Circle Draw',
                                                     rows=5, cols=3)

    calibrator = projector.Calibrator();

    s1 = ecto.Strand()
    main_display = highgui.imshow("rgb show", name="rgb", waitKey=5, strand=s1)
    graph = [
                sync["image"] >> im2mat_rgb["image"],
                im2mat_rgb["image"] >> main_display[:],
                sync["depth"] >> im2mat_depth["image"],
                im2mat_depth["image"] >> highgui.imshow("depth show", name="depth", waitKey= -1, strand=s1)[:],
                im2mat_rgb["image"] >> invert[:],
                invert[:] >> circle_detector["input"],
                im2mat_rgb["image"] >> brg2rgb["input"],
                brg2rgb["out"] >> circle_drawer['input'],
                circle_detector['out', 'found'] >> circle_drawer['points', 'found'],
                circle_drawer["out"] >> highgui.imshow("pattern show", name="pattern", waitKey= -1, strand=s1)[:],
#                sync["image","depth"] >> pattern_detection['image', 'depth'],
#                pattern_detection['points'] >> projection_estimator['points']
            ]

    print "Press 's' to capture a view."

    # add the display of the pattern
    video_display = highgui.imshow('pattern',
                               name='video_cap', waitKey=2, maximize=False, autoSize=True)
    graph += [pattern_draw['pattern'] >> video_display['input'],
              pattern_draw['points'] >> calibrator['pattern'],
              circle_detector['out', 'found'] >> calibrator['points', 'found'],
              camera_info['K'] >> calibrator['K'],
              main_display['out'] >> calibrator['trigger'],
              im2mat_depth["image"] >> calibrator['depth'] ]


    plasm = ecto.Plasm()
    plasm.connect(graph)

    # display DEBUG data if needed
    if DEBUG:
        print plasm.viz()
        ecto.view_plasm(plasm)

    # execute the pipeline
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()


if __name__ == "__main__":
    ecto_ros.init(sys.argv, "ecto_node")
    do_projector()
