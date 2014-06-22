#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage, Inc. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.


import ecto

from ecto_opencv.highgui import imshow
from ecto_opencv.calib import DepthTo3d
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.rgbd import DepthCleaner, ComputeNormals, DepthSwapper, DrawNormals, RgbdNormalsTypes
from ecto.opts import run_plasm, scheduler_options
from ecto_image_pipeline.io.source import create_source

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Computes the odometry between frames.')
    scheduler_options(parser.add_argument_group('Scheduler'))
    options = parser.parse_args()

    return options

if __name__ == '__main__':
    options = parse_args()

    plasm = ecto.Plasm()

    #setup the input source, grayscale conversion
    from ecto_openni import VGA_RES, FPS_30
    source = create_source('image_pipeline','OpenNISource',image_mode=VGA_RES,image_fps=FPS_30)
    #from ecto_openni import SXGA_RES, FPS_15
    #source = create_source('image_pipeline','OpenNISource',image_mode=SXGA_RES,image_fps=FPS_15)
    depth_cleaner = DepthCleaner()

    #connect up the pose_est
    connections = [ source['depth'] >> imshow(name='depth')[:],
                    source['depth'] >> depth_cleaner['image'],
                    depth_cleaner['image'] >> imshow(name='depth_clean')[:] ]

    # connect to normal finders

    depth_to_3d = DepthTo3d()
    depth_swapper = DepthSwapper()
    compute_normals = {}
    draw_normals = {}
    for i in [0, 1]:
        compute_normals[i] = ComputeNormals(method=RgbdNormalsTypes.FALS)
        draw_normals[i] = DrawNormals(step=20)

    connections += [ source['depth'] >> depth_to_3d['depth'],
                     source['K_depth'] >> depth_to_3d['K'],
                    ]
    for i in [0, 1]:
        if i==0:
            connections += [ depth_to_3d['points3d'] >> compute_normals[i]['points3d'] ]
        else:
            connections += [ depth_to_3d['points3d'] >> depth_swapper['points3d'],
                             depth_cleaner['image'] >> depth_swapper['depth'],
                             depth_swapper['points3d'] >> compute_normals[i]['points3d'] ]
        connections += [ source['K_depth'] >> compute_normals[i]['K'] ]
        connections += [ compute_normals[i]['normals'] >> draw_normals[i]['normals'],
                         depth_to_3d['points3d'] >> draw_normals[i]['points3d'],
                         source['image', 'K_depth'] >> draw_normals[i]['image', 'K'],
                         draw_normals[i]['normal_intensity'] >> imshow(name=str(i),waitKey=1)[:] ]

    plasm.connect(connections)

    run_plasm(options, plasm, locals=vars())
