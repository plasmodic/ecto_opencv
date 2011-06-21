/*
 * twoDToThreeD.cpp
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

struct twoDToThreeD
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
    inputs.declare<cv::Mat>("K", "The calibration matrix");
    inputs.declare<cv::Mat>("depth", "The depth image");
    outputs.declare<std::vector<cv::Point3f> >("pts", "The output 3d points");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    std::cout << "twoDToThreeD" << std::endl;
    return 0;
  }
};

void wrap_twoDToThreeD() {
  ecto::wrap<twoDToThreeD>(
      "twoDToThreeD",
      "An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).");
}
