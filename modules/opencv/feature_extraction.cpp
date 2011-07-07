/*
 * feature_extraction.cpp

 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */
#include <cstddef>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ecto/ecto.hpp>

#include "feature_extraction.h"

cv::FeatureDetector* FeatureDescriptorFinder::createDetector(const std::string& extractor_type,
                                                             const std::map<std::string, double> &params)
{
  cv::FeatureDetector* fd = 0;
  if (!extractor_type.compare("FAST"))
  {
    if (params.end() != params.find("threshold"))
      fd = new cv::FastFeatureDetector(params.at("threshold")/*threshold*/, true/*nonmax_suppression*/);
    else
      fd = new cv::FastFeatureDetector();
  }
  else if (!extractor_type.compare("STAR"))
  {
    if (params.end() != params.find("threshold"))
      fd = new cv::StarFeatureDetector(16/*max_size*/, (int)params.at("threshold")/*response_threshold*/,
                                       10/*line_threshold_projected*/, 8/*line_threshold_binarized*/,
                                       5/*suppress_nonmax_size*/);
    else
      fd = new cv::StarFeatureDetector();
  }
  else if (!extractor_type.compare("SIFT"))
  {
    fd = new cv::SiftFeatureDetector(cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                     cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  }
  else if (!extractor_type.compare("SURF"))
  {
    float threshold = params.at("threshold");

    fd = new cv::SurfFeatureDetector(threshold/*hessian_threshold*/, 5/*octaves*/, 4/*octave_layers*/);
  }
  else if (!extractor_type.compare("MSER"))
  {
    float threshold = params.at("threshold");

    fd = new cv::MserFeatureDetector(5/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.25f/*max_variation*/,
                                     0.2/*min_diversity*/, 200/*max_evolution*/, threshold/*area_threshold*/,
                                     0.003/*min_margin*/, 5/*edge_blur_size*/);
  }
  else if (!extractor_type.compare("GFTT"))
  {
    float threshold = params.at("threshold");

    fd = new cv::GoodFeaturesToTrackDetector(1000/*maxCorners*/, threshold/*qualityLevel*/, 1./*minDistance*/,
                                             3/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/);
  }
  else if (!extractor_type.compare("ORB"))
  {
    cv::ORB::CommonParams orb_params;
    orb_params.n_levels_ = params.at("octaves");
    orb_params.scale_factor_ = params.at("scale_factor");

    fd = new cv::OrbFeatureDetector(params.at("n_features"), orb_params);
  }
  else
    assert(0);
  return fd;
}

FeatureDescriptorFinder* FeatureDescriptorFinder::create(const std::string &json_params)
{
  // First, parse the JSON string
  boost::property_tree::ptree params;
  boost::property_tree::read_json(json_params, params);

  std::string descriptor_type = params.get<std::string>("descriptor");
  std::string feature_type = params.get<std::string>("feature");
  std::string combination_type = params.get<std::string>("combination");

  // Deal with some specific cases of feature/descriptor combination
  if (combination_type == "ORB")
  {
    cv::ORB::CommonParams orb_params;
    orb_params.scale_factor_ = params.get<float>("feature_params.scale_factor");
    orb_params.n_levels_ = params.get<unsigned int>("feature_params.octaves");
    return new cv::OrbExtractor(orb_params, params.get<unsigned int>("feature_params.n_features"));
  }

  FeatureDescriptorFinder* fe = 0;
  cv::Ptr<cv::FeatureDetector> detector;

  if (params.detector_type == "DynamicSTAR")
  {
    detector = new cv::DynamicAdaptedFeatureDetector(new cv::StarAdjuster(), params.detector_params.at("min_features"),
                                                     params.detector_params.at("max_features"), 200);
  }
  else if (params.detector_type == "DynamicSURF")
    detector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(), params.detector_params.at("min_features"),
                                                     params.detector_params.at("max_features"), 200);
  else
  {
    FeatureExtractionParams new_params = params;
    if ((params.descriptor_type == "ORB") && (params.extractor_type == "multi-scale"))
    {
      new_params.detector_params.at("octaves") = 1;
      new_params.detector_params.at("scale_factor") = 1;
    }

    detector = FeatureDescriptorFinder::createDetector(params.detector_type, new_params.detector_params);
  }

  // Define the extractor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (params.descriptor_type == "ORB")
  {
    cv::ORB::CommonParams orb_params;
    if (params.extractor_type == "multi-scale")
    {
      orb_params.n_levels_ = 1;
      orb_params.scale_factor_ = 1;
    }
    else
    {
      orb_params.n_levels_ = params.extractor_params.at("octaves");
      orb_params.scale_factor_ = params.extractor_params.at("scale_factor");
    }

    extractor = new cv::OrbDescriptorExtractor(orb_params);
  }
  else
  {
    extractor = cv::DescriptorExtractor::create(params.descriptor_type);
    if (extractor.empty())
      throw std::runtime_error("bad extractor");
  }

  if (params.extractor_type == "multi-scale")
    fe = new MultiscaleExtractor(detector, extractor, params.extractor_params.at("octaves"));
  else if (params.extractor_type == "sequential")
    fe = new SequentialExtractor(detector, extractor);
  else if (params.extractor_type == "ORB")
  {
    cv::ORB::CommonParams orb_params;
    orb_params.scale_factor_ = params.extractor_params.at("scale_factor");
    orb_params.n_levels_ = params.extractor_params.at("octaves");
    fe = new OrbExtractor(orb_params, params.detector_params.at("n_features"));
  }

  return fe;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MultiscaleExtractor::MultiscaleExtractor(const cv::Ptr<cv::FeatureDetector>& d,
                                         const cv::Ptr<cv::DescriptorExtractor>& e, int n_octaves) :
    detector_(d), extractor_(e), n_octaves_(n_octaves)
{
}

void MultiscaleExtractor::detectAndExtract(const cv::Mat & image_in, const cv::Mat & mask_in,
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

OrbExtractor::OrbExtractor(cv::ORB::CommonParams params, int n_desired_features) :
    params_(params), n_desired_features_(n_desired_features)
{
  orb_ = cv::ORB(n_desired_features_, params);
}

void OrbExtractor::detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                    cv::Mat &descriptors) const
{
  orb_(image, mask, keypoints, descriptors);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SequentialExtractor::SequentialExtractor(const cv::Ptr<cv::FeatureDetector>& d,
                                         const cv::Ptr<cv::DescriptorExtractor>& e) :
    detector_(d), extractor_(e)
{

}
void SequentialExtractor::detectAndExtract(const cv::Mat & image, const cv::Mat & mask,
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
void FileExtractor::detectAndExtract(const cv::Mat & image, const cv::Mat & mask, std::vector<cv::KeyPoint> & keypoints,
                                     cv::Mat &descriptors) const
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
                           "The path of the file containing YAML parameters for feature/descriptor extraction.");
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
    std::string json_params_ = params.get<std::string>("json_params");
    FeatureExtractionParams feature_descriptor_params;
    if (params_.empty())
    {
      // Read them from the file
    }

    feature_descriptor_params.read(json_params_);
    feature_descriptor_ = FeatureDescriptorFinder::create(feature_descriptor_params);
  }

  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    const cv::Mat &image = inputs.get<cv::Mat>("image");
    const cv::Mat &mask = inputs.get<cv::Mat>("mask");

    std::vector<cv::KeyPoint>& keypoints = outputs.get<std::vector<cv::KeyPoint> >("keypoints");
    std::vector<cv::KeyPoint>& descriptors = outputs.get<std::vector<cv::KeyPoint> >("descriptors");

    cv::FastFeatureDetector fd(thresh_, true);
    fd.detect(in, kpts, mask);
    return 0;
  }
private:
  /** The parameters (stored in a JSON string) */
  std::string json_params_;
  boost::shared_ptr<FeatureDescriptor> feature_descriptor_;
};

void wrap_FeatureDescriptor()
{
  ecto::wrap<FeatureDescriptor>("FeatureDescriptor", "Compute features and descriptors for an image.");
}
