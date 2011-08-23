#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
using ecto::tendrils;
namespace calib
{
  struct DepthTo3d
  {
    /**
     *
     * @param K
     * @param depth a subregion of a matrix.
     * @param points3d
     * @param roi the roi that create the subregion.
     */
    static void
    depthTo3d(const cv::Mat& K, const cv::Mat& depth, cv::Mat& points3d)
    {
      //allocate points.
      cv::Mat_<float> points = cv::Mat_<float>(depth.size().area(), 3);
      cv::Mat_<float>::iterator sp_begin = points.begin();
      //grab camera params
      float fx = K.at<float>(0, 0);
      float fy = K.at<float>(1, 1);
      float cx = K.at<float>(0, 2);
      float cy = K.at<float>(1, 2);
      // Create 3D points in one go.
      cv::Size depth_size = depth.size();
      if (depth.depth() == CV_16U)
      {
        const uint16_t* r;
        for (int v = 0; v < depth_size.height; v++)
        {
          r = depth.ptr<uint16_t>(v,0);
          for (int u = 0; u < depth_size.width; u++)
          {
            uint16_t fpz = *(r++);
            if (fpz == std::numeric_limits<uint16_t>::max())
            {
              std::cout << "fpz max:" << fpz << std::endl;
            }
            float z =  fpz / 1000.0f;
            *(sp_begin++) = (u - cx) * z / fx;
            *(sp_begin++) = (v - cy) * z / fy;
            *(sp_begin++) = z;
          }
        }
      }
      else if (depth.depth() == CV_32F)
      {
        const float* r;
        for (int v = 0; v < depth_size.height; v++)
        {
          r = depth.ptr<float>(v,0);

          for (int u = 0; u < depth_size.width; u++)
          {
            float z = *(r++);
            if(std::isfinite(z))
            {
              *(sp_begin++) = (u - cx) * z / fx;
              *(sp_begin++) = (v - cy) * z / fy;
              *(sp_begin++) = z;
            }else
            {
              *(sp_begin++) = z;
              *(sp_begin++) = z;
              *(sp_begin++) = z;
            }
          }
        }
      }
      points3d = points; //return Nx3 matrix
    }

    typedef std::vector<cv::Point2f> points_t;
    static void
    declare_params(tendrils& p)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("K", "The calibration matrix").required(true);
      inputs.declare<cv::Mat>("depth", "The depth image").required(true);
      outputs.declare<cv::Mat>("points3d", "The 3d points.");

    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat K, depth;
      inputs["K"] >> K;
      inputs["depth"] >> depth;
      K.clone().convertTo(K, CV_32F);
      cv::Mat points3d;
      depthTo3d(K, depth, points3d);
      outputs["points3d"] << points3d;
      return 0;
    }
  };

}
using namespace calib;
ECTO_CELL(calib, DepthTo3d, "DepthTo3d", "Converts depth to 3d points.")
