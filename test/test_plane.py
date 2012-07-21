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
    plane_finder = PlaneFinder(error=0.03)
    depth_to_3d = DepthTo3d()
    compute_normals1 = ComputeNormals(method=RgbdNormalsTypes.FALS)
    draw_normals1 = DrawNormals(step=20)
    plane_drawer = PlaneDrawer()

    #plasm.connect(source['image'] >> rgb2gray ['image'])

    #connect up the pose_est
    connections = [ source['depth_raw'] >> depth_to_3d['depth'],
                    source['K'] >> depth_to_3d['K'],
                    source['image'] >> imshow(name='original',waitKey=1)[:]
                    ]
    connections += [ source['K'] >> compute_normals1['K'],
                    depth_to_3d['points3d'] >> compute_normals1['points3d']
                    ]
    connections += [ compute_normals1['normals'] >> draw_normals1['normals'],
                     depth_to_3d['points3d'] >> draw_normals1['points3d'],
                     source['image', 'K'] >> draw_normals1['image', 'K'],
                     draw_normals1['normal_intensity'] >> imshow(name='normalsFALS',waitKey=1)[:] ]
    connections += [ depth_to_3d['points3d'] >> plane_finder['point3d'],
                     compute_normals1['normals'] >> plane_finder['normals'] ]
    connections += [ plane_finder['masks'] >> plane_drawer['masks'],
                     plane_finder['planes'] >> plane_drawer['planes'],
                     source['K'] >> plane_finder['K'],
                     source['image'] >> plane_drawer['image'],
                     plane_drawer['image'] >> imshow(name='plane')[:],
                     source['image'] >> imshow(name='original', waitKey=10)[:]]
    plasm.connect(connections)

    run_plasm(options, plasm, locals=vars())
