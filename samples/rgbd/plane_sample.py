#!/usr/bin/env python

import ecto

from ecto_opencv.highgui import imshow
from ecto_opencv.calib import DepthTo3d
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.rgbd import ComputeNormals, DrawNormals, RgbdNormalsTypes
from ecto.opts import run_plasm, scheduler_options
from ecto_image_pipeline.io.source import create_source
from ecto_opencv.rgbd import PlaneFinder, PlaneDrawer

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Find a plane in an RGBD image.')
    scheduler_options(parser.add_argument_group('Scheduler'))
    options = parser.parse_args()

    return options


if __name__ == '__main__':
    options = parse_args()

    plasm = ecto.Plasm()

    #setup the input source, grayscale conversion
    from ecto_openni import SXGA_RES, FPS_15, VGA_RES, FPS_30
    source = create_source('image_pipeline','OpenNISource',image_mode=VGA_RES,image_fps=FPS_30)
    rgb2gray = cvtColor('Grayscale', flag=Conversion.RGB2GRAY)
    depth_to_3d = DepthTo3d()
    compute_normals = {}
    draw_normals = {}
    plane_drawer = {}
    plane_finder = {}
    normal_types = [ RgbdNormalsTypes.LINEMOD, RgbdNormalsTypes.FALS ]
    for type in normal_types:
        compute_normals[type] = ComputeNormals(method=type)
        draw_normals[type] = DrawNormals(step=20)
        plane_drawer[type] = PlaneDrawer()
        plane_finder[type] = PlaneFinder(threshold=0.03,sensor_error_a=0.0075)

    #plasm.connect(source['image'] >> rgb2gray ['image'])

    #connect up the pose_est
    connections = [ source['depth'] >> depth_to_3d['depth'],
                    source['K_depth'] >> depth_to_3d['K'],
                    source['image'] >> imshow(name='original',waitKey=1)[:]
                    ]

    # compute the normals
    for type in [RgbdNormalsTypes.FALS, RgbdNormalsTypes.SRI]:
        if type in normal_types:
            connections += [ depth_to_3d['points3d'] >> compute_normals[type]['points3d'] ]
    for type in [RgbdNormalsTypes.LINEMOD ]:
        if type in normal_types:
            connections += [ source['depth'] >> compute_normals[type]['points3d'] ]

    # send the camera calibration parameters
    for type in normal_types:
        connections += [ source['K_depth'] >> compute_normals[type]['K'] ]

    # draw the normals
    for type in normal_types:
        connections += [ compute_normals[type]['normals'] >> draw_normals[type]['normals'],
                         depth_to_3d['points3d'] >> draw_normals[type]['points3d'],
                         source['image', 'K_depth'] >> draw_normals[type]['image', 'K'],
                         draw_normals[type]['normal_intensity'] >> imshow(name=str(type),waitKey=1)[:] ]

    # find the planes
    for type in normal_types:
        connections += [ depth_to_3d['points3d'] >> plane_finder[type]['points3d'],
                         compute_normals[type]['normals'] >> plane_finder[type]['normals'] ]
        connections += [ plane_finder[type]['masks'] >> plane_drawer[type]['masks'],
                         source['K_depth'] >> plane_finder[type]['K'],
                         source['image'] >> plane_drawer[type]['image'],
                         plane_drawer[type]['image'] >> imshow(name='plane'+str(type))[:] ]
    connections += [ source['image'] >> imshow(name='original', waitKey=10)[:] ]
    plasm.connect(connections)

    run_plasm(options, plasm, locals=vars())
