#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using ecto::tendrils;
using namespace cv;

/* BOILER_PLATE_MODULE
 struct MyModule
 {
 static void declare_params(tendrils& params);
 static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
 void configure(tendrils& params);
 int process(const tendrils& in, tendrils& out);
 };
 */
namespace line_mod
{
struct ColorMod
{
  void computeColorOrder(const cv::Mat &Iin, cv::Mat &Iord)
  {
    //     GaussianBlur(Iin, Iord, Size(5, 5), 2, 2);
    if (Iin.size() != Iord.size() || Iord.type() != CV_8UC1) //Make sure Iord is the right size
    {
      Iord.create(Iin.size(), CV_8UC1);
    }

    for (int y = 0; y < Iin.rows; y++)
    {
      const uchar *b = Iin.ptr<uchar> (y);
      const uchar *g = b + 1;
      const uchar *r = b + 2;
      uchar *o = Iord.ptr<uchar> (y);
      for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
      {
        int B = *b, R = *r, G = *g, Bt = B - thresh_gt, Rt = R - thresh_gt, Gt =
            G - thresh_gt, Wt = 255 - thresh_bw;
        if (Wt < B && Wt < G && Wt < R) //white
          *o = 64;
        else if (thresh_bw > B && thresh_bw > R && thresh_bw > G) //Black
          *o = 128;
        else if (Bt > R && Bt > G) //B
          *o = 1;
        else if (Gt > R && Gt > B) //G
          *o = 4;
        else if (Rt > G && Rt > B) //R
          *o = 16;
        else if (Bt > R && Bt < G) //Turquiose
          *o = 2;
        else if (Gt > B && Gt < R) //Yellow
          *o = 8;
        else if (Rt > G && Rt < B) //Purple
          *o = 32;
        else
          *o = 0;
      }
    }
  }
  static void declare_params(tendrils& p)
  {
    p.declare<int> ("thresh_gt", "Threshhold used when comparing colors.", 40);
    p.declare<int> ("thresh_bw",
        "Threshhold for deciding when the color is black or white", 35);
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("image", "An input image. RGB");
    outputs.declare<cv::Mat> ("output", "A binarized color image.");
  }

  void config(tendrils& params)
  {
    thresh_gt = params.get<int> ("thresh_gt");
    thresh_bw = params.get<int> ("thresh_bw");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("image");
    cv::Mat& output = outputs.get<cv::Mat> ("output");
    computeColorOrder(image, output);
    return 0;
  }

  int thresh_gt, thresh_bw;

};

struct ColorDebug
{
  void idealize_colors(cv::Mat input, cv::Mat& output)
  {
    //decode binary images here:
    output = input;
  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("input",
        "A binarized color image from the color mod module");
    out.declare<cv::Mat> ("output",
        "A psychodelic looking image for debugging the color idealization");
  }
  int process(const tendrils& in, tendrils& out)
  {
    idealize_colors(in.get<cv::Mat> ("input"), out.get<cv::Mat> ("output"));
    return 0;
  }
};
}

BOOST_PYTHON_MODULE(line_mod)
{
  using namespace line_mod;
  ecto::wrap<ColorMod>("ColorMod");
  ecto::wrap<ColorDebug>("ColorDebug");
}
