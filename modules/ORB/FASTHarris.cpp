#include "FASTHarris.h"

#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <boost/foreach.hpp>


namespace FASTHarris
{template<typename PatchType, typename SumType>
  inline float harris(const cv::Mat& patch, float k, const std::vector<int> &dX_offsets,
                      const std::vector<int> &dY_offsets)
  {
    float a = 0, b = 0, c = 0;

    static cv::Mat_<SumType> dX(9, 7), dY(7, 9);
    SumType * dX_data = reinterpret_cast<SumType*> (dX.data), *dY_data = reinterpret_cast<SumType*> (dY.data);
    SumType * dX_data_end = dX_data + 9 * 7;
    PatchType * patch_data = reinterpret_cast<PatchType*> (patch.data);
    int two_row_offset = 2 * patch.step1();
    std::vector<int>::const_iterator dX_offset = dX_offsets.begin(), dY_offset = dY_offsets.begin();
    // Compute the differences
    for (; dX_data != dX_data_end; ++dX_data, ++dY_data, ++dX_offset, ++dY_offset)
    {
      *dX_data = (SumType)(*(patch_data + *dX_offset)) - (SumType)(*(patch_data + *dX_offset - 2));
      *dY_data = (SumType)(*(patch_data + *dY_offset)) - (SumType)(*(patch_data + *dY_offset - two_row_offset));
    }

    // Compute the Scharr result
    dX_data = reinterpret_cast<SumType*> (dX.data);
    dY_data = reinterpret_cast<SumType*> (dY.data);
    for (size_t v = 0; v <= 6; v++, dY_data += 2)
    {
      for (size_t u = 0; u <= 6; u++, ++dX_data, ++dY_data)
      {
        //float weight = 10 - std::sqrt((v - 4) * (v - 4) + (u - 4) * (u - 4));
        float weight = 1.0f;// / (9.0 * 9.0);
        float Ix = 3 * (*dX_data + *(dX_data + 14)) + 10 * (*(dX_data + 7));
        float Iy = 3 * (*dY_data + *(dY_data + 2)) + 10 * (*(dY_data + 1));

        Ix *= weight;
        Iy *= weight;
        a += Ix * Ix;
        b += Iy * Iy;
        c += Ix * Iy;
      }
    }

    return ((a * b - c * c) - (k * ((a + b) * (a + b))));
  }

HarrisResponse::HarrisResponse(const cv::Mat& image, double k) :
  image(image), k(k)
{
  // Compute the offsets for the Harris corners once and for all
  dX_offsets_.resize(7 * 9);
  dY_offsets_.resize(7 * 9);
  std::vector<int>::iterator dX_offsets = dX_offsets_.begin(), dY_offsets = dY_offsets_.begin();
  unsigned int image_step = image.step1();
  for (size_t y = 0; y <= 6 * image_step; y += image_step)
  {
    int dX_offset = y + 2, dY_offset = y + 2 * image_step;
    for (size_t x = 0; x <= 6; ++x)
    {
      *(dX_offsets++) = dX_offset++;
      *(dY_offsets++) = dY_offset++;
    }
    for (size_t x = 7; x <= 8; ++x)
      *(dY_offsets++) = dY_offset++;
  }

  for (size_t y = 7 * image_step; y <= 8 * image_step; y += image_step)
  {
    int dX_offset = y + 2;
    for (size_t x = 0; x <= 6; ++x)
      *(dX_offsets++) = dX_offset++;
  }
}

void HarrisResponse::operator()(std::vector<cv::KeyPoint>& kpts) const
{
  BOOST_FOREACH(cv::KeyPoint & kpt, kpts)
        {
          // make sure the keypoint and its neighborhood is fully in the image
          if ((kpt.pt.x - 4 < 0) || (kpt.pt.y - 4 < 0) || (kpt.pt.x + 4 >= image.cols) || (kpt.pt.y + 4 >= image.rows))
            kpt.response = 0;
          else
          {
            cv::Mat patch = image(cv::Rect(kpt.pt.x - 4, kpt.pt.y - 4, 9, 9));

            // Compute the response
#if 0
            cv::Mat_<float> Ix(9, 9), Iy(9, 9);

            cv::Scharr(patch, Ix, CV_32F, 1, 0);
            cv::Scharr(patch, Iy, CV_32F, 0, 1);
            Ix = Ix / (9.0 * 9.0);
            Iy = Iy / (9.0 * 9.0);
            float a = 0, b = 0, c = 0;
            for (unsigned int y = 1; y <= 7; ++y)
            {
              for (unsigned int x = 1; x <= 7; ++x)
              {
                a += Ix(y, x) * Ix(y, x);
                b += Iy(y, x) * Iy(y, x);
                c += Ix(y, x) * Iy(y, x);
              }
            }
            //[ a c ]
            //[ c b ]
            float response = (float)((a * b - c * c) - k * ((a + b) * (a + b)));
#endif
            kpt.response = harris<uchar, int> (patch, k, dX_offsets_, dY_offsets_);
#if 0
            std::cout << response - kpt.response << std::endl;
#endif
          }
        }
}
void SimpleFASTHarris::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  //detect fast with a resonable threshhold, and nonmax
  cv::FastFeatureDetector fd(20, true);
  fd.detect(image, keypoints, mask);
  size_t n_features = desired_n_features_;
  //grab our harris response, operates in place on the keypoints
  HarrisResponse h(image);h(keypoints);
  //take only the top N if there are too many features
  if (keypoints.size() > n_features)
  {
    std::nth_element(keypoints.begin(), keypoints.begin() + n_features, keypoints.end(), keypointResponseGreater);
    keypoints.resize(n_features);
  }
}
}
#if 0
int main(int argc, char** argv){
  if(argc != 2)
  { 
    std::cerr << "usage:\n";
    std::cerr << argv[0] << " image_file" << std::endl;
    return -1;
  }
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  SimpleFASTHarris sfh(500);
  std::vector<cv::KeyPoint> kpts;
  sfh.detect(image,kpts);
  std::cout << "found " << kpts.size() << "kpts" << std::endl;
  cv::Mat draw_image;
  cv::drawKeypoints(image, kpts, draw_image, cv::Scalar::all(-1));
  cv::imshow("keypoints",draw_image);
  while((0xFF & cv::waitKey(0)) != 'q')
    std::cout << "press 'q' to quit" << std::endl;
}
#endif
