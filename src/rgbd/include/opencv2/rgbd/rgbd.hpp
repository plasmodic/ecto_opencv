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
   * a limit. For a float/double, we just check if it is a NaN
   * @param depth the depth to check for validity
   */
  CV_EXPORTS
  inline bool
  isValidDepth(const float & depth)
  {
    return cvIsNaN(depth);
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const double & depth)
  {
    return cvIsNaN(depth);
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const short int & depth)
  {
    return (depth != std::numeric_limits<short int>::min()) && (depth != std::numeric_limits<short int>::max());
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const unsigned short int & depth)
  {
    return (depth != std::numeric_limits<unsigned short int>::min())
        && (depth != std::numeric_limits<unsigned short int>::max());
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const int & depth)
  {
    return (depth != std::numeric_limits<int>::min()) && (depth != std::numeric_limits<int>::max());
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const unsigned int & depth)
  {
    return (depth != std::numeric_limits<unsigned int>::min()) && (depth != std::numeric_limits<unsigned int>::max());
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
    initialize_normals_impl(int rows, int cols, int depth, const cv::Mat & K, int window_size, int method) const;

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

  /** Object that can compute planes in an image
   */
  CV_EXPORTS
  class RgbdPlane: public cv::Algorithm
  {
  public:
    enum RGBD_PLANE_METHOD
    {
      RGBD_PLANE_METHOD_DEFAULT
    };

    RgbdPlane(RGBD_PLANE_METHOD method = RGBD_PLANE_METHOD_DEFAULT)
        :
          method_(method),
          block_size_(40),
          threshold_(0.01),
          sensor_error_a_(0),
          sensor_error_b_(0),
          sensor_error_c_(0)
    {
    }

    /** Find The planes in a depth image
     * @param depth image. If it has 3 channels, it is assumed to be 2d points
     * @param the normals for every point in the depth image
     * @param mask An image where each pixel is labeled with the plane it belongs to
     *        and 255 if it does not belong to any plane
     * @param the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0
     */
    void
    operator()(const cv::Mat & depth, const cv::Mat & normals, cv::Mat &mask,
               std::vector<cv::Vec4f> & plane_coefficients);

    /** Find The planes in a depth image but without doing a normal check, which is faster but less accurate
     * @param depth image. If it has 3 channels, it is assumed to be 2d points
     * @param mask An image where each pixel is labeled with the plane it belongs to
     *        and 255 if it does not belong to any plane
     * @param the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0
     */
    void
    operator()(const cv::Mat & depth, cv::Mat &mask, std::vector<cv::Vec4f> & plane_coefficients);

    AlgorithmInfo*
    info() const;
  private:
    /** The method to use to compute the planes */
    int method_;
    /** The size of the blocks to look at for a stable MSE */
    int block_size_;
    /** How far a point can be from a plane to belong to it (in meters) */
    double threshold_;
    /** coefficient of the sensor error with respect to the. All 0 by default but you want a=0.0075 for a Kinect */
    double sensor_error_a_, sensor_error_b_, sensor_error_c_;
  };

  // TODO 
  //      2) tests

  /** Base class for computation of odometry.
   */
  CV_EXPORTS class Odometry: public Algorithm
  {
  public:
    enum { ROTATION          = 1,
           TRANSLATION       = 2,
           RIGID_BODY_MOTION = 4
         };
    static inline float
    DEFAULT_MIN_DEPTH()
    {
      return 0.f;
    }
    static inline float
    DEFAULT_MAX_DEPTH()
    {
      return 4.f;
    }
    static inline float
    DEFAULT_MAX_DEPTH_DIFF()
    {
      return 0.07f;
    }
    static inline float
    DEFAULT_USED_POINTS_PART()
    {
      return 0.07f;
    }

    /** Method to compute transformation between two neighboring frames.
     * Some odometry algorithms do not used some data of frames (eg. ICP does not use images).
     * In such case corresponding arguments can be set as empty cv::Mat.
     * @param image0 Image data of first frame
     * @param depth0 Depth data of first frame
     * @param mask0 Mask that sets which pixels have to be used from first frame
     * @param image1 Image data of second frame
     * @param depth1 Depth data of second frame
     * @param mask1 Mask that sets which pixels have to be used from second frame
     * @param Rt Resulting transformation from first frame to second one (rigid body motion)
     * @param initRt Initial transformation from first frame to second one (optional)
     */
    bool
    compute(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
            const Mat& mask1, Mat& Rt, const Mat& initRt = Mat()) const;
  protected:
    virtual void
    checkParams() const = 0;
    virtual void
    checkFrames(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1) const = 0;
    virtual bool
    computeImpl(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1, const Mat& initRt, Mat& Rt) const = 0;
  };

  /** Odometry based on the paper "Real-Time Visual Odometry from Dense RGB-D Images", 
   * F. Steinbucker, J. Strum, D. Cremers, ICCV, 2011.
   */
  CV_EXPORTS class RgbdOdometry: public Odometry
  {
  public:
    RgbdOdometry();
    /** Constructor.
     * @param cameraMatrix Camera matrrix
     * @param minDepth Pixels with depth less than minDepth will not be used
     * @param maxDepth Pixels with depth larger than maxDepth will not be used
     * @param maxDepthDiff Correspondences between pixels of two given frames will be filtered out
     *                     if their depth difference is larger than maxDepthDiff
     * @param iterCounts Count of iterations on each pyramid level.
     * @param minGradientMagnitudes For each pyramid level the pixels will be filtered out
     *                              if they have gradient magnitude less than minGradientMagnitudes[level].
     */
    RgbdOdometry(const Mat& cameraMatrix, float minDepth = DEFAULT_MIN_DEPTH(), float maxDepth = DEFAULT_MAX_DEPTH(),
                 float maxDepthDiff = DEFAULT_MAX_DEPTH_DIFF(), const vector<int>& iterCounts = vector<int>(),
                 const vector<float>& minGradientMagnitudes = vector<float>(), int transformType=RIGID_BODY_MOTION);

    AlgorithmInfo*
    info() const;

  protected:
    virtual void
    checkParams() const;
    virtual void
    checkFrames(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1) const;
    virtual bool
    computeImpl(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1, const Mat& initRt, Mat& Rt) const;
    // params
    Mat cameraMatrix;
    // Some params have commented desired type. It's due to cv::AlgorithmInfo::addParams does not support it now.
    /*float*/
    double minDepth, maxDepth, maxDepthDiff;
    /*vector<int>*/
    Mat iterCounts;
    /*vector<float>*/
    Mat minGradientMagnitudes;

    int transformType;
  };

  /** Odometry based on the paper "KinectFusion: Real-Time Dense Surface Mapping and Tracking", 
   * Richard A. Newcombe, Andrew Fitzgibbon, at al, SIGGRAPH, 2011.
   */
  class ICPOdometry: public Odometry
  {
  public:
    ICPOdometry();
    /** Constructor.
     * @param cameraMatrix Camera matrrix
     * @param minDepth Pixels with depth less than minDepth will not be used
     * @param maxDepth Pixels with depth larger than maxDepth will not be used
     * @param maxDepthDiff Correspondences between pixels of two given frames will be filtered out
     *                     if their depth difference is larger than maxDepthDiff
     * @param pointsPart The method uses a random pixels subset of size frameWidth x frameHeight x pointsPart
     * @param iterCounts Count of iterations on each pyramid level.
     */
    ICPOdometry(const Mat& cameraMatrix, float minDepth = DEFAULT_MIN_DEPTH(), float maxDepth = DEFAULT_MAX_DEPTH(),
                float maxDepthDiff = DEFAULT_MAX_DEPTH_DIFF(), float pointsPart = DEFAULT_USED_POINTS_PART(),
                const vector<int>& iterCounts = vector<int>(), int transformType=RIGID_BODY_MOTION);

    AlgorithmInfo*
    info() const;

  protected:
    virtual void
    checkParams() const;
    virtual void
    checkFrames(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1) const;
    virtual bool
    computeImpl(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1, const Mat& initRt, Mat& Rt) const;
    // params
    Mat cameraMatrix;
    // Some params have commented desired type. It's due to cv::AlgorithmInfo::addParams does not support it now.
    /*float*/
    double minDepth, maxDepth, maxDepthDiff;
    /*float*/
    double pointsPart;
    /*vector<int>*/
    Mat iterCounts;

    mutable vector<cv::Ptr<cv::RgbdNormals> > normalComputers;

    int transformType;
  };

  /** Odometry that merges RgbdOdometry and ICPOdometry by minimize sum of their energy functions.
   */
  class RgbdICPOdometry: public Odometry
  {
  public:
    RgbdICPOdometry();
    /** Constructor.
     * @param cameraMatrix Camera matrrix
     * @param minDepth Pixels with depth less than minDepth will not be used
     * @param maxDepth Pixels with depth larger than maxDepth will not be used
     * @param maxDepthDiff Correspondences between pixels of two given frames will be filtered out
     *                     if their depth difference is larger than maxDepthDiff
     * @param pointsPart The method uses a random pixels subset of size frameWidth x frameHeight x pointsPart
     * @param iterCounts Count of iterations on each pyramid level.
     * @param minGradientMagnitudes For each pyramid level the pixels will be filtered out
     *                              if they have gradient magnitude less than minGradientMagnitudes[level].
     */
    RgbdICPOdometry(const Mat& cameraMatrix, float minDepth = DEFAULT_MIN_DEPTH(), float maxDepth = DEFAULT_MAX_DEPTH(),
                    float maxDepthDiff = DEFAULT_MAX_DEPTH_DIFF(), float pointsPart = DEFAULT_USED_POINTS_PART(),
                    const vector<int>& iterCounts = vector<int>(),
                    const vector<float>& minGradientMagnitudes = vector<float>(),
                    int transformType=RIGID_BODY_MOTION);

    AlgorithmInfo*
    info() const;

  protected:
    virtual void
    checkParams() const;
    virtual void
    checkFrames(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1) const;
    virtual bool
    computeImpl(const Mat& image0, const Mat& depth0, const Mat& mask0, const Mat& image1, const Mat& depth1,
                const Mat& mask1, const Mat& initRt, Mat& Rt) const;
    // params
    Mat cameraMatrix;
    // Some params have commented desired type. It's due to cv::AlgorithmInfo::addParams does not support it now.
    /*float*/
    double minDepth, maxDepth, maxDepthDiff;
    /*float*/
    double pointsPart;
    /*vector<int>*/
    Mat iterCounts;
    /*vector<float>*/
    Mat minGradientMagnitudes;

    mutable vector<cv::Ptr<cv::RgbdNormals> > normalComputers;

    int transformType;
  };

// TODO Depth interpolation
// Curvature
// Get rescaleDepth return dubles if asked for
} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */
