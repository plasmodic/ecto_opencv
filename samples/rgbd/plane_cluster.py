#!/usr/bin/env python

import ecto

from ecto_opencv.highgui import imshow
from ecto_opencv.calib import DepthTo3d
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.rgbd import ComputeNormals, ClusterDrawer, DrawNormals, OnPlaneClusterer, RgbdNormalsTypes
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
    compute_normals = ComputeNormals(method=RgbdNormalsTypes.FALS)
    plane_drawer = PlaneDrawer()
    plane_finder = PlaneFinder(threshold=0.03,sensor_error_a=0.0075, min_size=10000)

    #connect up the pose_est
    connections = [ source['depth'] >> depth_to_3d['depth'],
                    source['K_depth'] >> depth_to_3d['K'],
                    source['image'] >> imshow(name='original')[:]
                    ]

    # compute the normals
    connections += [ depth_to_3d['points3d'] >> compute_normals['points3d'] ]

    # send the camera calibration parameters
    connections += [ source['K_depth'] >> compute_normals['K'] ]

    # find the planes
    connections += [ depth_to_3d['points3d'] >> plane_finder['points3d'],
                         compute_normals['normals'] >> plane_finder['normals'] ]
    connections += [ plane_finder['masks'] >> plane_drawer['masks'],
                         source['K_depth'] >> plane_finder['K'],
                         source['image'] >> plane_drawer['image'],
                         plane_drawer['image'] >> imshow(name='plane')[:] ]
    connections += [ source['image'] >> imshow(name='original')[:] ]

    # find the clusters
    on_plane_clusterer = OnPlaneClusterer()
    connections += [ depth_to_3d['points3d'] >> on_plane_clusterer['points3d'],
                     plane_finder['masks'] >> on_plane_clusterer['masks'],
                     plane_finder['planes'] >> on_plane_clusterer['planes'],
                   ]

    # draw the clusters
    cluster_drawer = ClusterDrawer()
    connections += [ on_plane_clusterer['clusters2d'] >> cluster_drawer['clusters2d'],
                     source['image'] >> cluster_drawer['image'],
                     cluster_drawer['image'] >> imshow(name='clusters', waitKey=10)[:]
                   ]
    plasm.connect(connections)

    run_plasm(options, plasm, locals=vars())
