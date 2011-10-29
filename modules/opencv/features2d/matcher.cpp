#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;

struct HammingOperator
{
  typedef unsigned char ValueType;

  //! important that this is signed as weird behavior happens
  // in BruteForce if not
  typedef int ResultType;

  unsigned int
  operator()(const unsigned char* a, const unsigned char* b, int size)
  {
#if __GNUC__
    unsigned int result = 0;
    {
      //for portability just use unsigned long -- and use the __builtin_popcountll (see docs for __builtin_popcountll)
      typedef unsigned long long pop_t;
      const size_t modulo = size % sizeof(pop_t);
      const pop_t * a2 = reinterpret_cast<const pop_t*>(a);
      const pop_t * b2 = reinterpret_cast<const pop_t*>(b);

      if (size == sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2));
      if (size == 2 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)));
      if (size == 3 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)))
               + __builtin_popcountll((*(a2 + 2)) ^ (*(b2 + 2)));
      if (size == 4 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)))
               + __builtin_popcountll((*(a2 + 2)) ^ (*(b2 + 2)))
               + __builtin_popcountll((*(a2 + 3)) ^ (*(b2 + 3)));

      const pop_t * a2_end = a2 + (size / sizeof(pop_t));

      for (; a2 != a2_end; ++a2, ++b2)
        result += __builtin_popcountll((*a2) ^ (*b2));

      if (modulo)
      {
        //in the case where size is not divisible by sizeof(size_t)
        //need to mask off the bits at the end
        pop_t a_final = 0, b_final = 0;
        memcpy(&a_final, a2, modulo);
        memcpy(&b_final, b2, modulo);
        result += __builtin_popcountll(a_final ^ b_final);
      }
    }
    return result;
#else
    return cv::HammingLUT()(a,b,size);
#endif
  }
};

typedef std::vector<cv::KeyPoint> kpts_t;
typedef std::vector<cv::DMatch> matches_t;
struct Matcher
{
  static void
  declare_params(tendrils& p)
  {
  }
  static void
  declare_io(const tendrils& p, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("train", "Test descriptors.");
    inputs.declare<cv::Mat>("test", "Train descriptors.");
    outputs.declare<matches_t>("matches", "The descriptor matches.");
  }
  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    cv::Mat train, test;
    inputs["train"] >> train;
    inputs["test"] >> test;
    cv::BruteForceMatcher<HammingOperator> matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(test, train, matches);
    outputs["matches"] << matches;
    return ecto::OK;
  }
};

ECTO_CELL(features2d, Matcher, "Matcher", "A feature descriptor matcher.");

