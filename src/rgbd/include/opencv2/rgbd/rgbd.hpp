/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef __OPENCV_RGBD_HPP__
#define __OPENCV_RGBD_HPP__

#ifdef __cplusplus

#include <limits.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

namespace cv
{
  /** Checks if the value is a valid depth. For CV_16U or CV_16S, the convention is to be invalid if it is
   * a limit. For a float, we just check if it is a NaN
   * @param depth
   * @param in_K
   * @param in_points
   * @param points3d
   */
  CV_EXPORTS
  template<typename T>
  bool
  isValidDepth(const T & depth)
  {
    return (depth != std::numeric_limits<T>::min()) && (depth != std::numeric_limits<T>::max());
  }

  /** Object that can compute the normals in an image.
   * It is an object as it can cache data for speed efficiency
   */
  CV_EXPORTS
  class RgbdNormals: public Algorithm
  {
  public:
    enum RGBD_NORMALS_METHOD
    {
      RGBD_NORMALS_METHOD_SRI, RGBD_NORMALS_METHOD_FALS
    };

    RgbdNormals()
        :
          rows_(0),
          cols_(0),
          depth_(0),
          K_(cv::Mat()),
          window_size_(0),
          method_(RGBD_NORMALS_METHOD_FALS),
          rgbd_normals_impl_(0)
    {
    }

    /** Constructor
     */
    RgbdNormals(int rows, int cols, int depth, const cv::Mat & K, int window_size = 5, int method =
        RGBD_NORMALS_METHOD_FALS);

    ~RgbdNormals();

    AlgorithmInfo*
    info() const;

    /** Given a set of 3d points in a depth image, compute the normals at each point.
     * @param points a rows x cols x 3 matrix
     * @param window_size the window size on which to compute the derivatives
     * @return normals a rows x cols x 3 matrix
     */
    cv::Mat
    operator()(const cv::Mat &points) const;

  protected:
    void
    initialize_normals_impl(int rows, int cols, int depth, const cv::Mat & K, int window_size,
                            int  method) const;

    int rows_, cols_, depth_;
    cv::Mat K_;
    int window_size_;
    int method_;
    mutable void* rgbd_normals_impl_;
  };

  /**
   * @param depth the depth image
   * @param K
   * @param in_points the list of xy coordinates
   * @param points3d the resulting 3d points
   */
  CV_EXPORTS
  void
  depthTo3dSparse(const cv::Mat& depth, const cv::Mat& in_K, const cv::InputArray in_points, cv::Mat& points3d);

  /** Converts a depth image to an organized set of 3d points.
   * The coordinate system is x pointing left, y down and z away from the camera
   * @param depth the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
   *              (as done with the Microsoft Kinect), otherwise, if given as CV_32F, it is assumed in meters)
   * @param K The calibration matrix
   * @param points3d the resulting 3d points
   * @param mask the mask of the points to consider (can be empty)
   */
  CV_EXPORTS
  void
  depthTo3d(const cv::Mat& depth, const cv::Mat& K, cv::Mat& points3d, const cv::Mat& mask = cv::Mat());

  /** If the input image is of type CV_16UC1 (like the Kinect one), the image is converted to floats, divided
   * by 1000 to get a depth in meters, and the values 0 are converted to std::numeric_limits<float>::quiet_NaN()
   * Otherwise, the image is simply converted to floats
   * @param in the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
   *              (as done with the Microsoft Kinect), it is assumed in meters)
   * @param the desired output depth (floats or double)
   * @param out The rescaled float depth image
   */
  CV_EXPORTS
  void
  rescaleDepth(const cv::Mat& in, int depth, cv::Mat& out);

  /** Object that can compute the normals in an image.
   * It is an object as it can cache data for speed efficiency
   */
  CV_EXPORTS
  class RgbdPlane // : public cv::Algorithm
  {
  public:
    enum RGBD_PLANE_METHOD
    {
      RGBD_PLANE_METHOD_DEFAULT
    };

    RgbdPlane();

    RgbdPlane(int rows, int cols, int depth, const cv::Mat & K, int window_size, RGBD_PLANE_METHOD method =
        RGBD_PLANE_METHOD_DEFAULT);

    /** Find
     * @param depth image. If it has 3 channels, it is assumed to be 2d points
     * @param mask An image where each pixel is labeled with the plane it belongs to
     */
    void
    operator()(const cv::Mat & depth, cv::Mat &mask, std::vector<cv::Vec4f> & plane_coefficients);

    void
    operator()(const cv::Mat & depth, const cv::Mat & normals, cv::Mat &mask,
               std::vector<cv::Vec4f> & plane_coefficients);
  private:
    RgbdNormals rgbd_normals_;
  };

// ICP (Maria)
  // TODO 1) refactor the interface (make it as a class?),
  //      2) add comments
  //      3) put image and depth filtering into the function (?),
  //      4) do not check and process rgb or depth if they are not needed
  //         (eg. rgb processing in case of ICP_ODOMETRY or some normals in case of RGBD_ODOMETRY)
  enum
  {
    RGBD_ODOMETRY = 1, ICP_ODOMETRY = 2, MERGED_ODOMETRY = RGBD_ODOMETRY + ICP_ODOMETRY
  };

  CV_EXPORTS bool
  RGBDICPOdometry(cv::Mat& Rt, const cv::Mat& initRt, const cv::Mat& image0, const cv::Mat& _depth0,
                  const cv::Mat& validMask0, const cv::Mat& image1, const cv::Mat& _depth1, const cv::Mat& validMask1,
                  const cv::Mat& cameraMatrix, std::vector<cv::Ptr<cv::RgbdNormals> >& normalComputers, float minDepth =
                      0.f,
                  float maxDepth = 4.f, float maxDepthDiff = 0.07f, const std::vector<int>& iterCounts =
                      std::vector<int>(),
                  const std::vector<float>& minGradientMagnitudes = std::vector<float>(), float icpPointsPart = 0.07f,
                  int methodType = RGBD_ODOMETRY);

// TODO Depth interpolation
// Curvature
// Get rescaleDepth return dubles if asked for
} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */
