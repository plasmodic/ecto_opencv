#!/usr/bin/env python
import ecto
import ecto_ros, ecto_sensor_msgs
import sys
import subprocess
import yaml

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def do_ecto():
    bagname = sys.argv[1]
    proc = subprocess.Popen(['rosbag','info','-k','topics','-y',bagname],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    stdout,stderr = proc.communicate()
    #should exit without error
    if len(stderr) != 0:
        print stderr
        sys.exit(-1)
    result = yaml.load(stdout)
    counts = {info['topic']:info['messages'] for info in result}
    #test that the counts are the same.
    assert counts['/camera/rgb/image_color'] == counts['/camera/depth/image']
    assert counts['/camera/rgb/image_color'] != 0
        
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   )
    
    bagreader = ecto_ros.BagReader('Bag Ripper',
                                    baggers=baggers,
                                    bag=bagname,
                                  )
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    counter_rgb = ecto.Counter()
    counter_depth = ecto.Counter()

    graph = [
                bagreader["image"] >> im2mat_rgb["image"],
                bagreader["depth"] >> im2mat_depth["image"],
                im2mat_rgb[:] >> counter_rgb[:],
                im2mat_depth[:] >> counter_depth[:]

            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    #ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
    print "expecting count:", counts['/camera/rgb/image_color']
    print "RGB count:",counter_rgb.outputs.count
    print "Depth count:", counter_depth.outputs.count
    assert counts['/camera/rgb/image_color'] == counter_rgb.outputs.count
    assert counts['/camera/depth/image']  == counter_depth.outputs.count
    #sched = ecto.schedulers.Singlethreaded(plasm)
    #sched.execute()
if __name__ == "__main__":
    do_ecto()
