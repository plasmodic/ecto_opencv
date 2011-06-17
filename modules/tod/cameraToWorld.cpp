/*
 * cameraToWorld.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

using ecto::tendrils;

struct cameraToWorld
{
  static void declare_params(tendrils& p)
  {
    p.declare<int>("n_features", "The number of desired features", 1000);
    p.declare<int>("n_levels", "The number of scales", 3);
    p.declare<float>("scale_factor", "The factor between scales", 1.2);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector<cv::KeyPoint> >("pts", "The keypoints");
    inputs.declare<cv::Mat>("R", "The rotation matrix");
    inputs.declare<cv::Mat>("T", "The translation vector");
    outputs.declare<std::vector<cv::KeyPoint> >("pts", "The keypoints");
  }

  void configure(tendrils& params)
  {
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    std::cout << "cameraToWorld" << std::endl;
    return 0;
  }
};

void wrap_cameraToWorld()
{
  ecto::wrap<cameraToWorld>(
      "cameraToWorld",
      "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
}
