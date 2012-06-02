/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <stdexcept>

#include <opencv2/rgbd/rgbd.hpp>

#include "test_precomp.hpp"

cv::Point3f
rayPlaneIntersection(cv::Point2f uv, const cv::Mat& centroid, const cv::Mat& normal, const cv::Mat_<float>& Kinv);

cv::Vec3f
rayPlaneIntersection(const cv::Vec3d& uv1, double centroid_dot_normal, const cv::Vec3d& normal,
                     const cv::Matx33d& Kinv);
cv::Vec3f
rayPlaneIntersection(const cv::Vec3d& uv1, double centroid_dot_normal, const cv::Vec3d& normal, const cv::Matx33d& Kinv)
{

  cv::Matx31d L = Kinv * uv1; //a ray passing through camera optical center
  //and uv.
  L = L * (1.0 / cv::norm(L));
  double LdotNormal = L.dot(normal);
  double d;
  if (std::fabs(LdotNormal) > 1e-9)
  {
    d = centroid_dot_normal / LdotNormal;
  }
  else
  {
    d = 1.0;
    std::cout << "warning, LdotNormal nearly 0! " << LdotNormal << std::endl;
    std::cout << "contents of L, Normal: " << cv::Mat(L) << ", " << cv::Mat(normal) << std::endl;
  }
  cv::Vec3f xyz(d * L(0), d * L(1), d * L(2));
  return xyz;
}

cv::Point3f
rayPlaneIntersection(cv::Point2f uv, const cv::Mat& centroid, const cv::Mat& normal, const cv::Mat_<float>& Kinv)
{
  cv::Matx33d dKinv(Kinv);
  cv::Vec3d dNormal(normal);
  return rayPlaneIntersection(cv::Vec3d(uv.x, uv.y, 1), centroid.dot(normal), dNormal, dKinv);
}

const int W = 640;
const int H = 480;
int window_size = 5;
float focal_length = 525;
float cx = W / 2.f + 0.5f;
float cy = H / 2.f + 0.5f;

cv::Mat K = (cv::Mat_<double>(3, 3) << focal_length, 0, cx, 0, focal_length, cy, 0, 0, 1);
cv::Mat Kinv = K.inv();

static cv::RNG rng;
struct Plane
{

  cv::Vec3d n, p;
  double p_dot_n;
  Plane()
  {
    n[0] = rng.uniform(-0.5, 0.5);
    n[1] = rng.uniform(-0.5, 0.5);
    n[2] = -0.3; //rng.uniform(-1.f, 0.5f);
    n = n / cv::norm(n);
    set_d(rng.uniform(-2.0, 0.6));
  }

  void
  set_d(float d)
  {
    p = cv::Vec3d(0, 0, d / n[2]);
    p_dot_n = p.dot(n);
  }

  cv::Vec3f
  intersection(float u, float v, const cv::Matx33f& Kinv) const
  {
    return rayPlaneIntersection(cv::Vec3d(u, v, 1), p_dot_n, n, Kinv);
  }
};

void
gen_points_3d(std::vector<Plane>& planes_out, cv::Mat& points3d, cv::Mat& normals, int n_planes)
{
  std::vector<Plane> planes;
  for (int i = 0; i < n_planes; i++)
  {
    Plane px;
    for (int j = 0; j < 1; j++)
    {
      px.set_d(rng.uniform(-3.f, -0.5f));
      planes.push_back(px);
    }
  }
  cv::Mat_<cv::Vec3f> outp(cv::Size(W, H));
  cv::Mat_<cv::Vec3f> outn(cv::Size(W, H));
  // n  ( r - r_0) = 0
  // n * r_0 = d
  //
  // r_0 = (0,0,0)
  // r[0]
  for (int v = 0; v < H; v++)
  {
    for (int u = 0; u < W; u++)
    {
      Plane plane = planes[(u / float(W)) * planes.size()];
      outp(v, u) = plane.intersection(u, v, Kinv);
      outn(v, u) = plane.n;
    }
  }
  planes_out = planes;
  points3d = outp;
  normals = outn;
}

void
testit(cv::Mat points3d, cv::Mat in_ground_normals, const cv::RgbdNormals & normals_computer, float thresh)
{
  cv::Mat in_normals = normals_computer(points3d);

  cv::Mat_<cv::Vec3f> normals, ground_normals;
  in_normals.convertTo(normals, CV_32FC3);
  in_ground_normals.convertTo(ground_normals, CV_32FC3);

  float err = 0;
  for (int y = 0; y < normals.rows; ++y)
    for (int x = 0; x < normals.cols; ++x)
    {
      cv::Vec3f vec1 = normals(y, x), vec2 = ground_normals(y, x);
      vec1 = vec1 / cv::norm(vec1);
      vec2 = vec2 / cv::norm(vec2);

      float dot = vec1.dot(vec2);
      // Just for rounding errors
      if (std::abs(dot) < 1)
        err += std::asin(dot);
    }

  cv::Mat diff = cv::abs(ground_normals - normals);
  cv::Scalar mean, stddev;
  cv::meanStdDev(diff, mean, stddev);
  double nmean = cv::norm(mean);
  ASSERT_LE(nmean, thresh) << "mean diff: " << nmean << " thresh: " << thresh << std::endl;
}

class CV_RgbdNormalsTest: public cvtest::BaseTest
{
public:
  CV_RgbdNormalsTest()
  {
  }
  ~CV_RgbdNormalsTest()
  {
  }
protected:
  void
  run(int)
  {
    try
    {
      /*
       // K from a VGA Kinect
       cv::Mat K = (cv::Mat_<float>(3, 3) << 525., 0., 319.5, 0., 525., 239.5, 0., 0., 1.);
       K = (cv::Mat_<float>(3, 3) << 1., 0., 0, 0., 1., 0, 0., 0., 1.);
       // Create a random plane
       cv::RNG rng;
       cv::Mat_<float> plane(1, 4);
       rng.fill(plane, cv::RNG::UNIFORM, -10, 10);

       // Create some 3d points on the plane
       int rows = 480, cols = 640;
       cv::Mat_<float> depth(rows, cols);
       {
       cv::Mat K_inv = K.inv();
       // Fix the scale of each point to fall on the plane
       for (int y = 0; y < rows; ++y)
       for (int x = 0; x < cols; ++x)
       {
       cv::Mat_<float> point = (K_inv * (cv::Mat_<float>(3, 1) << x, y, 1));
       point = point
       * (-plane(0, 3)
       / (point(0, 0) * plane(0, 0) + point(1, 0) * plane(0, 1) + point(2, 0) * plane(0, 2)));
       depth(y, x) = point(2, 0);
       }
       }

       // Compute the 3d points
       cv::Mat_ < cv::Vec3f > points3d;
       depthTo3d(depth, K, points3d);

       // Make sure the points belong to the plane
       float avg_diff = 0;
       for (int y = 0; y < rows; ++y)
       for (int x = 0; x < cols; ++x)
       {
       float err = 0;
       for (unsigned char i = 0; i < 3; ++i)
       err += points3d(y, x).val[i] * plane(0, i);
       avg_diff += std::abs(err + plane(0, 3));
       }

       // Verify the function works
       ASSERT_LE(avg_diff / rows / cols, 1e-4) << "Average error for ground truth is: " << (avg_diff / rows / cols);

       // Compute the normals
       std::vector<cv::RgbdNormals> normal_computers;
       normal_computers.push_back(cv::RgbdNormals(points3d.rows, points3d.cols, points3d.depth(), K, cv::RgbdNormals::RGBD_NORMALS_METHOD_FALS));
       normal_computers.push_back(cv::RgbdNormals(points3d.rows, points3d.cols, points3d.depth(), K, cv::RgbdNormals::RGBD_NORMALS_METHOD_SRI));

       for(unsigned int i=0;i<2;++i) {
       cv::Mat_ < cv::Vec3f > normals = normal_computers[i](points3d);

       avg_diff = 0;
       for (int y = 0; y < rows; ++y)
       for (int x = 0; x < cols; ++x)
       {
       cv::Vec3f normal1 = normals(y, x), normal2(plane(0, 0), plane(0, 1), plane(0, 2));
       normal1 = normal1 / cv::norm(normal1);
       normal2 = normal2 / cv::norm(normal2);

       avg_diff += std::min(cv::norm(normal1 - normal2), cv::norm(normal1 + normal2));
       }

       // Verify the function works
       ASSERT_LE(avg_diff / rows / cols, 1e-4) << "Case " << i<< " fails. Average error for normals is: " << (avg_diff / rows / cols);
       */

      for (unsigned char i = 1; i < 2; ++i)
      {
        cv::RgbdNormals::RGBD_NORMALS_METHOD method;
        if (i == 0)
          method = cv::RgbdNormals::RGBD_NORMALS_METHOD_FALS;
        if (i == 1)
          method = cv::RgbdNormals::RGBD_NORMALS_METHOD_SRI;

        for (unsigned char j = 0; j < 2; ++j)
        {
          int depth = (j % 2 == 0) ? CV_32F : CV_64F;

          cv::RgbdNormals normals_computer(H, W, depth, K, method);

          std::vector<Plane> plane_params;
          cv::Mat points3d, ground_normals;
          gen_points_3d(plane_params, points3d, ground_normals, 1);
          testit(points3d, ground_normals, normals_computer, 0.002); // 1 plane, continuous scene, very low error..
          for (int ii = 0; ii < 10; ii++)
          {
            gen_points_3d(plane_params, points3d, ground_normals, 3); //three planes
            testit(points3d, ground_normals, normals_computer, 0.02); // 3 discontinuities, more error expected.
          }
        }
      }

      //TODO test NaNs in data

    } catch (...)
    {
      ts->set_failed_test_info(cvtest::TS::FAIL_MISMATCH);
    }
    ts->set_failed_test_info(cvtest::TS::OK);
  }
};

TEST(Rgbd_Normals, compute)
{
  CV_RgbdNormalsTest test;
  test.safe_run();
}
