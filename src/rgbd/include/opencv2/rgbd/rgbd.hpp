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
     * @param rows the number of rows of the depth image normals will be computed on
     * @param cols the number of cols of the depth image normals will be computed on
     * @param depth the depth of the normals (only CV_32F or CV_64F)
     * @param K the calibration matrix to use
     * @param window_size the window size to compute the normals: can only be 1,3,5 or 7
     * @param method one of the methods to use: RGBD_NORMALS_METHOD_SRI, RGBD_NORMALS_METHOD_FALS
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
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param the normals for every point in the depth image
     * @param mask An image where each pixel is labeled with the plane it belongs to
     *        and 255 if it does not belong to any plane
     * @param the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0
     */
    void
    operator()(const cv::Mat & points3d, const cv::Mat & normals, cv::Mat &mask,
               std::vector<cv::Vec4f> & plane_coefficients);

    /** Find The planes in a depth image but without doing a normal check, which is faster but less accurate
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param mask An image where each pixel is labeled with the plane it belongs to
     *        and 255 if it does not belong to any plane
     * @param the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0
     */
    void
    operator()(const cv::Mat & points3d, cv::Mat &mask, std::vector<cv::Vec4f> & plane_coefficients);

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

  /** Object that contains a frame data that is possibly needed for the Odometry.
   * It's used for the efficiency (to pass precomputed data of the frame that participates
   * in the Odometry processing several times).
   */
  CV_EXPORTS struct OdometryFrameData
  {
    OdometryFrameData();
    OdometryFrameData(const Mat& image, const Mat& depth, const Mat& mask);
    void release();

    Mat image;
    Mat depth;
    Mat mask;
    Mat normals;

    vector<Mat> pyramidImage;
    vector<Mat> pyramidDepth;
    vector<Mat> pyramidMask;

    vector<Mat> pyramidCloud;

    vector<Mat> pyramid_dI_dx;
    vector<Mat> pyramid_dI_dy;
    vector<Mat> pyramidTexturedMask;

    vector<Mat> pyramidNormals;
    vector<Mat> pyramidNormalsMask;
  };

  /** Base class for computation of odometry.
   */
  CV_EXPORTS class Odometry: public Algorithm
  {
  public:

    /** A class of transformation*/
    enum { ROTATION          = 1,
           TRANSLATION       = 2,
           RIGID_BODY_MOTION = 4
         };

    /** These constants are used to set a type of cache which has to be prepared depending on the frame role:
     * srcFrame or dstFrame (see compute method). For the srcFrame and dstFrame different cache data may be required,
     * some part of a cache may be common for both frame roles.
     * @param CACHE_SRC The cache data for the srcFrame will be prepared.
     * @param CACHE_DST The cache data for the dstFrame will be prepared.
     * @param CACHE_ALL The cache data for both srcFrame and dstFrame roles will be computed.
     */
    enum { CACHE_SRC = 1,
           CACHE_DST = 2,
           CACHE_ALL = CACHE_SRC + CACHE_DST
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

    /** Method to compute a transformation from the source frame to the destination one.
     * Some odometry algorithms do not used some data of frames (eg. ICP does not use images).
     * In such case corresponding arguments can be set as empty cv::Mat.
     * @param srcImage Image data of the source frame
     * @param srcDepth Depth data of the source frame
     * @param srcMask Mask that sets which pixels have to be used from the source frame
     * @param dstImage Image data of the destination frame
     * @param dstDepth Depth data of the destination frame
     * @param dstMask Mask that sets which pixels have to be used from the destination frame
     * @param Rt Resulting transformation from the source frame to the destination one (rigid body motion):
                 dst_p = Rt * src_p, where dst_p is a homogeneous point in the destination frame and src_p is
                 homogeneous point in the source frame,
                 Rt is 4x4 matrix of CV_64FC1 type.
     * @param initRt Initial transformation from the source frame to the destination one (optional)
     */
    bool
    compute(const Mat& srcImage, const Mat& srcDepth, const Mat& srcMask,
            const Mat& dstImage, const Mat& dstDepth, const Mat& dstMask,
            Mat& Rt, const Mat& initRt = Mat()) const;

    /** One more method to compute a transformation from the source frame to the destination one.
     * It is designed to save on computing the frame data (image pyramids, normals, etc.).
     */
    bool
    compute(OdometryFrameData& srcFrame, OdometryFrameData& dstFrame, Mat& Rt, const Mat& initRt = Mat()) const;

    /** Prepare a cache for the frame. The function checks the precomputed/passed data (throws the error if this data
     * does not satisfy) and computes all remaining cache data needed for the frame. Returned size is a resolution
     * of the prepared frame.
     * @param odometry The odometry which will process the frame.
     * @param cacheType The cache type: CACHE_SRC, CACHE_DST or CACHE_ALL.
     */
    virtual Size prepareFrameData(OdometryFrameData& frame, int cacheType) const = 0;

  protected:
    virtual void
    checkParams() const = 0;

    virtual bool
    computeImpl(const OdometryFrameData& srcFrame, const OdometryFrameData& dstFrame, Mat& Rt, const Mat& initRt) const = 0;
  };

  /** Odometry based on the paper "Real-Time Visual Odometry from Dense RGB-D Images", 
   * F. Steinbucker, J. Strum, D. Cremers, ICCV, 2011.
   */
  CV_EXPORTS class RgbdOdometry: public Odometry
  {
  public:
    RgbdOdometry();
    /** Constructor.
     * @param cameraMatrix Camera matrix
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

    virtual Size prepareFrameData(OdometryFrameData& frame, int cacheType) const;

    AlgorithmInfo*
    info() const;

  protected:
    virtual void
    checkParams() const;

    virtual bool
    computeImpl(const OdometryFrameData& srcFrame, const OdometryFrameData& dstFrame, Mat& Rt, const Mat& initRt) const;

    // Some params have commented desired type. It's due to cv::AlgorithmInfo::addParams does not support it now.
    /*float*/
    double minDepth, maxDepth, maxDepthDiff;
    /*vector<int>*/
    Mat iterCounts;
    /*vector<float>*/
    Mat minGradientMagnitudes;

    Mat cameraMatrix;
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
     * @param cameraMatrix Camera matrix
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

    virtual Size prepareFrameData(OdometryFrameData& frame, int cacheType) const;

    AlgorithmInfo*
    info() const;

  protected:
    virtual void
    checkParams() const;

    virtual bool
    computeImpl(const OdometryFrameData& srcFrame, const OdometryFrameData& dstFrame, Mat& Rt, const Mat& initRt) const;

    // Some params have commented desired type. It's due to cv::AlgorithmInfo::addParams does not support it now.
    /*float*/
    double minDepth, maxDepth, maxDepthDiff;
    /*float*/
    double pointsPart;
    /*vector<int>*/
    Mat iterCounts;

    Mat cameraMatrix;
    int transformType;

    mutable cv::Ptr<cv::RgbdNormals> normalsComputer;
  };

  /** Odometry that merges RgbdOdometry and ICPOdometry by minimize sum of their energy functions.
   */

  class RgbdICPOdometry: public Odometry
  {
  public:
    RgbdICPOdometry();
    /** Constructor.
     * @param cameraMatrix Camera matrix
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

    virtual Size prepareFrameData(OdometryFrameData& frame, int cacheType) const;

    AlgorithmInfo*
    info() const;

  protected:
    virtual void
    checkParams() const;

    virtual bool
    computeImpl(const OdometryFrameData& srcFrame, const OdometryFrameData& dstFrame, Mat& Rt, const Mat& initRt) const;

    // Some params have commented desired type. It's due to cv::AlgorithmInfo::addParams does not support it now.
    /*float*/
    double minDepth, maxDepth, maxDepthDiff;
    /*float*/
    double pointsPart;
    /*vector<int>*/
    Mat iterCounts;
    /*vector<float>*/
    Mat minGradientMagnitudes;

    Mat cameraMatrix;
    int transformType;

    mutable cv::Ptr<cv::RgbdNormals> normalsComputer;
  };
  
  /** Warp the image: compute 3d points from the depth, transform them using given transformation, 
   * then project color point cloud to an image plane. 
   * This function can be used to visualize results of the Odometry algorithm.
   * @param image The image (of CV_8UC1 or CV_8UC3 type)
   * @param depth The depth (of type used in depthTo3d fuction)
   * @param Rt The transformation that will be applied to the 3d points computed from the depth
   * @param cameraMatrix Camera matrix
   * @param distCoeff Distortion coefficients
   * @param warpedImage The warped image.
   */
  CV_EXPORTS
  void
  warpImage(const Mat& image, const Mat& depth, const Mat& Rt, const Mat& cameraMatrix, const Mat& distCoeff,
            Mat& warpedImage);

// TODO Depth interpolation
// Curvature
// Get rescaleDepth return dubles if asked for
} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */
