#!/usr/bin/env python
import ecto, ecto_ros, ecto_sensor_msgs
import sys, subprocess, yaml, os

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def do_ecto():
    bagname = sys.argv[1]
    print "Bagname:", bagname
    assert 'ROS_ROOT' in os.environ
    cmd = [os.environ['ROS_ROOT'] + '/bin/rosbag','info','-k','topics','-y', bagname]
    print "CMD:", ' '.join(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout,stderr = proc.communicate()

    #should exit without error
    if len(stderr) != 0:
        print "ERROR from rosbag"
        print stderr
        sys.exit(-1)
    print "STDOUT:", stdout
    result = yaml.load(stdout)
    counts = {}
    for info in result:
        counts[info['topic']] = info['messages']

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
