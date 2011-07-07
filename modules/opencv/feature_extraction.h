/*
 * features.h
 *
 *  Created on: Nov 4, 2010
 *      Author: ethan
 */

#ifndef FEATURES_H_TOD_
#define FEATURES_H_TOD_

#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief interface to fill out a Features2d object with keypoints and descriptors
 */
class FeatureDescriptorFinder
{
public:
  virtual ~FeatureDescriptorFinder()
  {
  }
  virtual void detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                cv::Mat &descriptors) const = 0;
  static FeatureDescriptorFinder* create(const std::string &params);
private:
  static cv::FeatureDetector* createDetector(const std::string& detectorType,
                                             const std::map<std::string, double> &params);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Given feature detector and extractor, compute features and descriptors simultaneously
 *   at different scales
 *   \code
 #Pseudo code for multi scale features + descriptors
 N_OCTAVES = 3
 scale = 1.0
 eps = scale / pow(2, N_OCTAVES)
 do:
 detect(img,keypoints)
 extract(img,descriptions)
 for x in keypoints:
 x.scale = scale
 x.pt = x.pt * ( 1 / scale) #rescale the point so that its relative to the original image
 resize( img, img.size/2 )
 scale = scale / 2
 while(scale > eps)
 *   \endcode
 */
class MultiscaleExtractor : public FeatureDescriptorFinder
{
public:
  MultiscaleExtractor(const cv::Ptr<cv::FeatureDetector>& d, const cv::Ptr<cv::DescriptorExtractor>& e, int n_octaves);
  MultiscaleExtractor()
  {
  }
  template<typename Detector, typename Extractor>
    MultiscaleExtractor(const Detector& d, const Extractor& e, int n_octaves) :
        detector_(new Detector(d)), extractor_(new Extractor(e)), n_octaves_(n_octaves)
    {

    }

  virtual void detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                cv::Mat &descriptors) const;
private:
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  int n_octaves_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class OrbExtractor : public FeatureDescriptorFinder
{
public:
  OrbExtractor(cv::ORB::CommonParams params, int n_desired_features);
  void detectAndExtract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                        const cv::Mat& mask = cv::Mat()) const;
  virtual void detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                cv::Mat &descriptors) const;
private:
  cv::ORB::CommonParams params_;
  int n_desired_features_;
  mutable cv::ORB orb_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class SequentialExtractor : public FeatureDescriptorFinder
{
public:
  SequentialExtractor(const cv::Ptr<cv::FeatureDetector>& d, const cv::Ptr<cv::DescriptorExtractor>& e);
  SequentialExtractor()
  {
  }

  template<typename Detector, typename Extractor>
    SequentialExtractor(const Detector& d, const Extractor& e) :
        detector_(new Detector(d)), extractor_(new Extractor(e))
    {

    }
  virtual ~SequentialExtractor()
  {
  }

  virtual void detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                cv::Mat &descriptors) const;

private:
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief a nop extractor, for when you don't want to extract or detect.
 *
 */
class NOPExtractor : public FeatureDescriptorFinder
{
public:
  virtual ~NOPExtractor()
  {
  }
  virtual void detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                cv::Mat &descriptors) const
  {
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief a file based extractor, for when you have a file of detected features
 *
 */
class FileExtractor : public FeatureDescriptorFinder
{
public:
  FileExtractor(const std::string& f2dname);
  virtual ~FileExtractor()
  {
  }
  virtual void detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                cv::Mat &descriptors) const;
private:
  std::string f2dname_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** convert from a vector of 'keypoints' with N-d data to 2d  xy points
 */
void KeyPointsToPoints(const std::vector<cv::KeyPoint> & keypts, std::vector<cv::Point2f>& pts);
void PointsToKeyPoints(const std::vector<cv::Point2f>& keypts, std::vector<cv::KeyPoint>& pts);

#endif /* FEATURES_H_ */
