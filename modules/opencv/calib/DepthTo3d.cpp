#include <ecto/ecto.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
using ecto::tendrils;
namespace calib
{
  struct DepthTo3dSparse
  {
    /**
     * @param K
     * @param uv the X,Y coordinates of the points in the image (n_points by 2 matrix)
     * @param depth the full depth image
     * @param points3d the resulting 3d points
     */
    static void
    depthTo3d_sparse(const cv::Mat& K, const cv::Mat& in_uv, const cv::Mat& depth, cv::Mat& points3d)
    {
      unsigned int n_points = in_uv.rows;

      //grab camera params
      float fx = K.at<float>(0, 0);
      float fy = K.at<float>(1, 1);
      float cx = K.at<float>(0, 2);
      float cy = K.at<float>(1, 2);
      cv::Mat_<float> uv;
      in_uv.convertTo(uv, CV_32F);

      // Get the depth
      cv::Mat_<float> zs = cv::Mat_<float>(n_points, 1);
      if (depth.depth() == CV_16U)
      {
        for (unsigned int i = 0; i < n_points; ++i)
          zs(i, 0) = depth.at<uint16_t>(uv(i, 1), uv(i, 0));
        zs = zs / 1000.0f;
      }
      else if (depth.depth() == CV_32F)
      {
        for (unsigned int i = 0; i < n_points; ++i)
          zs(i, 0) = depth.at<float>(uv(i, 1), uv(i, 0));
      }

      // Store into the final points
      cv::Mat_<float> coordinates[3] =
      { (uv.col(0) - cx).mul(zs) / fx, (uv.col(1) - cy).mul(zs) / fy, zs };
      cv::Mat tmp_points;
      cv::merge(coordinates, 3, tmp_points);
      points3d = tmp_points.reshape(3, 1);
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
      inputs.declare<cv::Mat>("points", "The u,v coordinates (n_points by 2").required(true);
      inputs.declare<cv::Mat>("depth", "The depth image").required(true);
      outputs.declare<cv::Mat>("points3d", "The 3d points, 1 by n_points with 3 channels (x, y and z).");
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
      cv::Mat K;
      inputs["K"] >> K;
      const cv::Mat &depth = inputs.get<cv::Mat>("depth"), &uv = inputs.get<cv::Mat>("points");

      K.clone().convertTo(K, CV_32F);
      cv::Mat points3d;
      depthTo3d_sparse(K, uv, depth, points3d);
      outputs["points3d"] << points3d;
      return 0;
    }
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct DepthTo3d
  {
    /**
     * @param K
     * @param depth the depth image
     * @param mask the mask of the points to consider (can be empty)
     * @param points3d the resulting 3d points
     */
    static void
    depthTo3d(const cv::Mat& K, const cv::Mat& depth, const cv::Mat& mask, cv::Mat& points3d)
    {
      // Create 3D points in one go.
      cv::Size depth_size = depth.size();
      if (!mask.empty())
      {
        cv::Mat_<float> uv = cv::Mat_<float>(depth.size().area(), 2);
        cv::Mat_<uchar> uchar_mask = mask;
        if (mask.depth() != (CV_8U))
          mask.convertTo(uchar_mask, CV_8U);

        // Figure out the interesting indices
        unsigned int n_points = 0;
        const uchar* r;
        for (int v = 0; v < depth_size.height; v++)
        {
          r = uchar_mask.ptr<uchar>(v, 0);
          for (int u = 0; u < depth_size.width; u++, ++r)
          {
            if (*r)
            {
              uv(n_points, 0) = u;
              uv(n_points, 1) = v;
              ++n_points;
            }
          }
        }

        // Get the depth
        uv.reserve(n_points);

        DepthTo3dSparse::depthTo3d_sparse(K, uv, depth, points3d);
      }
      else
      {
        //allocate points.
        cv::Mat_<float> points = cv::Mat_<float>(depth.rows, depth.cols, 3);
        cv::Mat_<float>::iterator sp_begin = points.begin();
        //grab camera params
        float fx = K.at<float>(0, 0);
        float fy = K.at<float>(1, 1);
        float cx = K.at<float>(0, 2);
        float cy = K.at<float>(1, 2);

        if (depth.depth() == CV_16U)
        {
          const uint16_t* r;
          for (int v = 0; v < depth_size.height; v++)
          {
            r = depth.ptr<uint16_t>(v, 0);
            for (int u = 0; u < depth_size.width; u++)
            {
              uint16_t fpz = *(r++);
              if (fpz == std::numeric_limits<uint16_t>::max())
              {
                std::cout << "fpz max:" << fpz << std::endl;
              }
              float z = fpz / 1000.0f;
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
            r = depth.ptr<float>(v, 0);

            for (int u = 0; u < depth_size.width; u++)
            {
              float z = *(r++);
              if (std::isfinite(z))
              {
                *(sp_begin++) = (u - cx) * z / fx;
                *(sp_begin++) = (v - cy) * z / fy;
                *(sp_begin++) = z;
              }
              else
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
    }

    typedef std::vector<cv::Point2f> points_t;

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("K", "The calibration matrix").required(true);
      inputs.declare<cv::Mat>("depth", "The depth image").required(true);
      inputs.declare<cv::Mat>("mask", "The mask of the points to send back");
      outputs.declare<cv::Mat>(
          "points3d", "The 3d points, height by width (or 1 by n_points if mask) with 3 channels (x, y and z)");
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat K, depth, mask;
      inputs["K"] >> K;
      inputs["depth"] >> depth;
      inputs["mask"] >> mask;
      K.clone().convertTo(K, CV_32F);
      cv::Mat points3d;
      depthTo3d(K, depth, mask, points3d);

      outputs["points3d"] << points3d;
      return 0;
    }
  };
}
using namespace calib;
ECTO_CELL(calib, DepthTo3d, "DepthTo3d", "Converts depth to 3d points.")
ECTO_CELL(calib, DepthTo3dSparse, "DepthTo3dSparse", "Converts depth to 3d points.")
