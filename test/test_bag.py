#!/usr/bin/env python
import ecto
import ecto_ros, ecto_sensor_msgs
import sys

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def do_ecto():
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   )
    
    bagreader = ecto_ros.BagReader('Bag Ripper',
                                    baggers=baggers,
                                    bag=sys.argv[1],
                                  )
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    graph = [
                bagreader["image"] >> im2mat_rgb["image"],
                bagreader["depth"] >> im2mat_depth["image"],
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    #ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

    #sched = ecto.schedulers.Singlethreaded(plasm)
    #sched.execute()
if __name__ == "__main__":
    do_ecto()
