#include "test_precomp.hpp"

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
      // K from a VGA Kinect
      cv::Mat K = (cv::Mat_<float>(3, 3) << 525., 0., 319.5, 0., 525., 239.5, 0., 0., 1.);
      K = (cv::Mat_<float>(3, 3) << 1., 0., 320, 0., 1,240, 0., 0., 1.);

      // Create a random plane
      cv::RNG rng;
      cv::Mat_<float> plane(1, 4);
      rng.fill(plane, cv::RNG::UNIFORM, -10, 10);
      plane(0, 0) = 0;
      plane(0, 1) = 0;
      plane(0, 2) = 1;
      plane(0, 3) = -1;

      // Create some 3d points on the plane
      int rows = 480, cols = 640;
      cv::Mat_<float> depth(rows, cols);
      {
        cv::Mat_<float> depth_image = cv::Mat_<float>::ones(rows, cols);
        cv::Mat K_inv = K.inv();
        // Fix the scale of each point to fall on the plane
        for (int y = 0; y < rows; ++y)
          for (int x = 0; x < cols; ++x)
          {
            cv::Mat_<float> point = K_inv * (cv::Mat_<float>(3, 1) << x, y, 1);
            point = point
                    * (-plane(0, 3)
                       / (point(0, 0) * plane(0, 0) + point(1, 0) * plane(0, 1) + point(2, 0) * plane(0, 2)));
            depth(y, x) = point(2,0);
          }
      }

      // Compute the 3d points
      cv::Mat_<cv::Vec3f> points3d;
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
      ASSERT_LE(avg_diff/rows/cols, 1e-4) << "Average error for ground truth is: " << (avg_diff / rows / cols);

      // Compute the normals
      cv::RgbdNormals normal_computer(points3d.rows, points3d.cols, points3d.depth(), K);
      cv::Mat_<cv::Vec3f> normals = normal_computer(points3d);

      avg_diff = 0;
      for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
        {
          cv::Vec3f normal1 = normals(y, x), normal2(plane(0, 0), plane(0, 1), plane(0, 2));
          normal1 = normal1 / cv::norm(normal1);
          normal2 = normal2 / cv::norm(normal2);

          for (unsigned char i = 0; i < 3; ++i)
            std::cout << normal1[i] << " ";
          std::cout << " - ";
          for (unsigned char i = 0; i < 3; ++i)
            std::cout << normal2[i] << " ";
          std::cout << std::endl;

          avg_diff += std::min(cv::norm(normal1 - normal2), cv::norm(normal1 + normal2));
        }

      // Verify the function works
      ASSERT_LE(avg_diff/rows/cols, 1e-4) << "Average error for normals is: " << (avg_diff / rows / cols);
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
