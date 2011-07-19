/*
 * feature_extraction.cpp

 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */
#include <cstddef>
#include <iostream>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ecto/ecto.hpp>

#include "feature_extraction.h"

/** Convenience function to create a FeatureDetector from a property tree
 * @param feature_type anything valid in FeatureDetector
 * @param params the property tree to parse for parameters
 * @return
 */
cv::FeatureDetector* createDetector(const std::string& feature_type, const boost::property_tree::ptree & params)
{
  if (feature_type == "FAST")
  {
    return new cv::FastFeatureDetector(params.get<float>("threshold"), true);
  }
  else if (feature_type == "STAR")
  {
    return new cv::StarFeatureDetector(16/*max_size*/, params.get<int>("threshold")/*response_threshold*/,
                                       10/*line_threshold_projected*/, 8/*line_threshold_binarized*/,
                                       5/*suppress_nonmax_size*/);
  }
  else if (feature_type == "SIFT")
  {
    return new cv::SiftFeatureDetector(cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                       cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  }
  else if (feature_type == "SURF")
  {
    return new cv::SurfFeatureDetector(params.get<float>("threshold")/*hessian_threshold*/, 5/*octaves*/,
                                       4/*octave_layers*/);
  }
  else if (feature_type == "MSER")
  {
    return new cv::MserFeatureDetector(5/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.25f/*max_variation*/,
                                       0.2/*min_diversity*/, 200/*max_evolution*/,
                                       params.get<float>("threshold")/*area_threshold*/, 0.003/*min_margin*/,
                                       5/*edge_blur_size*/);
  }
  else if (feature_type == "GFTT")
  {
    return new cv::GoodFeaturesToTrackDetector(1000/*maxCorners*/, params.get<float>("threshold")/*qualityLevel*/,
                                               1./*minDistance*/, 3/*int _blockSize*/, true/*useHarrisDetector*/,
                                               0.04/*k*/);
  }
  else if (feature_type == "ORB")
  {
    cv::ORB::CommonParams orb_params;
    orb_params.n_levels_ = params.get<int>("octaves");
    orb_params.scale_factor_ = params.get<float>("scale_factor");

    return new cv::OrbFeatureDetector(params.get<int>("n_features"), orb_params);
  }
  else
    assert(0);
  return 0;
}

FeatureDescriptorFinder* FeatureDescriptorFinder::create(const std::string &json_params)
{
// First, parse the JSON string
  boost::property_tree::ptree params;
  std::stringstream ssparams;
  ssparams << json_params;
  boost::property_tree::read_json(ssparams, params);

  std::string combination_type = params.get<std::string>("combination");

// Deal with some specific cases of feature/descriptor combination
  if (combination_type == "ORB")
  {
    cv::ORB::CommonParams orb_params;
    orb_params.scale_factor_ = params.get<float>("feature_params.scale_factor", orb_params.scale_factor_);
    orb_params.n_levels_ = params.get<int>("feature_params.n_levels", orb_params.n_levels_);
    return new OrbFeatureDescriptor(orb_params, params.get<unsigned int>("feature_params.n_features"));
  }
  else if (combination_type == "SIFT")
  {
    cv::SIFT::CommonParams common_params;
    common_params.angleMode = params.get<float>("feature_params.angleMode");
    common_params.firstOctave = params.get<float>("feature_params.firstOctave");
    common_params.nOctaveLayers = params.get<float>("feature_params.nOctavesLayers");
    common_params.nOctaves = params.get<float>("feature_params.nOctaves");

    //TODO make parameters explicit
    cv::SIFT::DetectorParams detector_params;
    cv::SIFT::DescriptorParams descriptor_params;
    return new SiftFeatureDescriptor(common_params, detector_params, descriptor_params);
  }

// Figure out the feature detector
  std::string feature_type = params.get<std::string>("feature");
  boost::property_tree::ptree feature_params = params.get_child("feature_params");
  cv::Ptr<cv::FeatureDetector> detector;
  if (feature_type == "DynamicSTAR")
  {
    detector = new cv::DynamicAdaptedFeatureDetector(new cv::StarAdjuster(),
                                                     feature_params.get<unsigned int>("min_features"),
                                                     feature_params.get<unsigned int>("max_features"), 200);
  }
  else if (feature_type == "DynamicSURF")
    detector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(),
                                                     feature_params.get<unsigned int>("min_features"),
                                                     feature_params.get<unsigned int>("max_features"), 200);
  else
    detector = createDetector(feature_type, feature_params);

// Define the descriptor extractor
  std::string descriptor_type = params.get<std::string>("descriptor");
  boost::property_tree::ptree descriptor_params = params.get_child("descriptor_params");
  cv::Ptr<cv::DescriptorExtractor> extractor;
  extractor = cv::DescriptorExtractor::create(descriptor_type);
  if (extractor.empty())
    throw std::runtime_error("bad extractor");

// Figure out how to combine features and descriptors
  if (combination_type == "multiscale")
  {
    boost::property_tree::ptree combination_params = params.get_child("combination_params");
    return new MultiscaleExtractor(detector, extractor, combination_params.get<unsigned int>("octaves"));
  }
  else if (combination_type == "sequential")
    return new SequentialExtractor(detector, extractor);

  return 0;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MultiscaleExtractor::MultiscaleExtractor(const cv::Ptr<cv::FeatureDetector>& d,
                                         const cv::Ptr<cv::DescriptorExtractor>& e, int n_octaves) :
    detector_(d), extractor_(e), n_octaves_(n_octaves)
{
}

void MultiscaleExtractor::detect_and_extract(const cv::Mat & image_in, const cv::Mat & mask_in,
                                             std::vector<cv::KeyPoint> & keypoints_out, cv::Mat &descriptors_out) const
{
  int octaves = n_octaves_;
  cv::Mat image = image_in.clone();

  float scale_factor = sqrt(2);
  cv::Mat mask = mask_in.empty() ? cv::Mat() : mask_in.clone();

  float scale_x = 1.0f;
  float scale_y = 1.0f;
  for (int i = 0; i < octaves; i++)
  {
    std::vector<cv::KeyPoint> kpts;
    cv::Mat descriptors;
    detector_->detect(image, kpts, mask);
    extractor_->compute(image, kpts, descriptors);

    for (size_t j = 0; j < kpts.size(); j++)
    {
      kpts[j].pt.x *= scale_x;
      kpts[j].pt.y *= scale_y;
    }

    if (i < octaves - 1)
    {
      scale_x = image.cols / (image.cols / scale_factor);
      scale_y = image.rows / (image.rows / scale_factor);

      cv::Size n_size(image.cols / scale_factor, image.rows / scale_factor);
      cv::resize(image, image, n_size);
      if (!mask.empty())
        resize(mask, mask, n_size);
    }
    keypoints_out.insert(keypoints_out.end(), kpts.begin(), kpts.end());
    descriptors_out.push_back(descriptors);
#if 0
    imshow("octave", image);
    imshow("scaled mask", mask);
    waitKey(0);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OrbFeatureDescriptor::OrbFeatureDescriptor(cv::ORB::CommonParams params, int n_desired_features)
{
  orb_ = cv::ORB(n_desired_features, params);
}

void OrbFeatureDescriptor::detect_and_extract(const cv::Mat & image, const cv::Mat & mask,
                                              std::vector<cv::KeyPoint> & keypoints, cv::Mat &descriptors) const
{
  orb_(image, mask, keypoints, descriptors);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SiftFeatureDescriptor::SiftFeatureDescriptor(cv::SIFT::CommonParams common_params,
                                             cv::SIFT::DetectorParams detector_params,
                                             cv::SIFT::DescriptorParams descriptor_params)
{
  sift_ = cv::SIFT(common_params, detector_params, descriptor_params);
}

void SiftFeatureDescriptor::detect_and_extract(const cv::Mat & image, const cv::Mat & mask,
                                               std::vector<cv::KeyPoint> & keypoints, cv::Mat &descriptors) const
{
  sift_(image, mask, keypoints, descriptors);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SequentialExtractor::SequentialExtractor(const cv::Ptr<cv::FeatureDetector>& d,
                                         const cv::Ptr<cv::DescriptorExtractor>& e) :
    detector_(d), extractor_(e)
{

}
void SequentialExtractor::detect_and_extract(const cv::Mat & image, const cv::Mat & mask,
                                             std::vector<cv::KeyPoint> & keypoints, cv::Mat &descriptors) const
{
  detector_->detect(image, keypoints, mask);
  extractor_->compute(image, keypoints, descriptors);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FileExtractor::FileExtractor(const std::string& f2dname) :
    f2dname_(f2dname)
{

}
void FileExtractor::detect_and_extract(const cv::Mat & image, const cv::Mat & mask,
                                       std::vector<cv::KeyPoint> & keypoints, cv::Mat &descriptors) const
{
  cv::FileStorage fs(f2dname_, cv::FileStorage::READ);
  cv::read(fs["keypoints"], keypoints);
//    fs["keypoints"] >> features.keypoints;
  fs["descriptors"] >> descriptors;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void KeyPointsToPoints(const std::vector<cv::KeyPoint>& keypts, std::vector<cv::Point2f>& pts)
{
  pts.clear();
  pts.reserve(keypts.size());
  for (size_t i = 0; i < keypts.size(); i++)
  {
    pts.push_back(keypts[i].pt);
  }
}

void PointsToKeyPoints(const std::vector<cv::Point2f>& pts, std::vector<cv::KeyPoint>& kpts)
{
  kpts.clear();
  kpts.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); i++)
    kpts.push_back(cv::KeyPoint(pts[i], 6.0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct FeatureDescriptor
{
  static void declare_params(ecto::tendrils& p)
  {
    p.declare<std::string>("json_params", "The parameters of the feature/descriptor in JSON format.", "");
    p.declare<std::string>("param_file",
                           "The path of the file containing YAML parameters for feature/descriptor extraction.", "");
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<cv::Mat>("image", "The image to find features/descriptors on.");
    inputs.declare<cv::Mat>("mask", "The mask to apply to the image.");
    outputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The found keypoints.");
    outputs.declare<cv::Mat>("descriptors", "The matching descriptors.");
    outputs.declare<std::string>("params", "A YAML string that describes the parameters.");
  }

  void configure(ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    // First, try to get the parameter from the string
    json_params_ = params.get<std::string>("json_params");
    if (json_params_.empty())
    {
      // Read them from the file
      std::string param_file = params.get<std::string>("param_file");
      std::ifstream file(param_file.c_str(), std::ios::in);

      if (file.is_open())
      {
        std::stringstream ssparams;
        while (file.good())
        {
          std::string line;
          std::getline(file, line);
          ssparams << line;
        }
        json_params_ = ssparams.str();
        file.close();
      }
      else
      {
        throw std::runtime_error("file " + param_file + " does not exist");
      }
    }

    feature_descriptor_ = boost::shared_ptr<FeatureDescriptorFinder>(FeatureDescriptorFinder::create(json_params_));
  }

  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    const cv::Mat &image = inputs.get<cv::Mat>("image");
    const cv::Mat &mask = inputs.get<cv::Mat>("mask");

    std::vector<cv::KeyPoint>& keypoints = outputs.get<std::vector<cv::KeyPoint> >("keypoints");
    cv::Mat& descriptors = outputs.get<cv::Mat>("descriptors");

    feature_descriptor_->detect_and_extract(image, mask, keypoints, descriptors);

    return 0;
  }
private:
  /** The parameters (stored in a JSON string) */
  std::string json_params_;
  boost::shared_ptr<FeatureDescriptorFinder> feature_descriptor_;
};

ECTO_CELL(features2d, FeatureDescriptor, "FeatureDescriptor",
		"Compute features and descriptors for an image.");
