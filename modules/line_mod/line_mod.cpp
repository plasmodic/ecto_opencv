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
 void configure(tendrils& params, tendrils& inputs, tendrils& outputs);
 int process(const tendrils& in, tendrils& out);
 };
 */
namespace line_mod
{
struct ColorMod
{
  void computeColorOrder(cv::Mat Iin, cv::Mat &Iord)
  {
    GaussianBlur(Iin, Iin, Size(5, 5), 2, 2);
    if (Iin.size() != Iord.size() || Iord.type() != CV_8UC1) //Make sure Iord is the right size
    {
      Iord.create(Iin.size(), CV_8UC1);
    }

#ifdef usergb
    int foo = 0;
    for (int y = 0; y < Iin.rows; y++)
    {
      const uchar *b = Iin.ptr<uchar> (y);
      const uchar *g = b + 1;
      const uchar *r = b + 2;
      uchar *o = Iord.ptr<uchar> (y);
      for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
      {
#ifdef color_thresh_method
        int B = int(unsigned(*b)), R = int(unsigned(*r)), G = int(unsigned(*g)), Bt, Rt, Gt, Wt;
        if((Bt = B - thresh_gt) < 0) Bt = 0;
        if((Rt = R - thresh_gt) < 0) Rt = 0;
        if((Gt = G - thresh_gt) < 0) Gt = 0;
        if((Wt = 255 - thresh_bw) < 0) Wt = 0;
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
#elseif normed_color
        /*
         *   sqrt(2) = 1/1.414213562 rt2
         *   sqrt(3) = 1/1.732050808 rt3
         *             Binary color code
   Bit set        Color values  Category
   =======         =========     ======        B   G   R
     0   1         B>>R or G     "Blue"       [1,  0,  0  ]
     1   2         B~=G >> R     "Turquoise"  [rt2,rt2,0  ]
     2   4         G>>B or R     "Green"      [0,  1,  0  ]
     3   8        G~=R >> B      "Yellow"     [0,  rt2,rt2]
     4  16         R>>G or B     "Red"        [0,  0,   1 ]
     5  32         R~=B >> G     "Purple"     [rt2,0,  rt2]
     6  64         R,G,B >> 0    "White"      [rt3,rt3,rt3]  high mag
     7 128         R,G,B ~= 0    "Black"      [rt3,rt3,rt3]  low mag
         *
         */
        foo += 1; //db
      //Get L2 norm of color
      float B = float(unsigned(*b)), R = float(unsigned(*r)), G = float(unsigned(*g)), BB,RR,GG;
      BB = B*B; RR = R*R, GG = G*G;
      float c_norm = sqrt(BB+RR+GG);
      if(c_norm > 0.0) c_norm = 1.0/c_norm; else c_norm = 1.0;
      float nB = B*c_norm, nG = G*c_norm, nR = R*c_norm, mag, maxmag;
      uchar out;
      //dot product with color directions
      //BLUE
      out = 1;
      maxmag = nB;
      //Turquoise
      mag = 0.707106781f*nB + 0.707106781f*nG;
      if(mag > maxmag) { out = 2; maxmag = mag;}
      //Green
      if(nG > maxmag) { out = 4; maxmag = nG;}
      //Yellow
      mag = 0.707106781f*nG + 0.707106781f*nR;
      if(mag > maxmag) {out = 8; maxmag = mag;}
      //Red
      if(nR > maxmag) {out = 16; maxmag = nR;}
      //Purple
      mag = 0.707106781f*nB + 0.707106781f*nR;
      if(mag > maxmag) { out = 32; maxmag = mag;}
      //White or Black
      int cmag = R+G+B;
      if((cmag < 3*thresh_bw)||(cmag > 765 - 3*thresh_bw))
      {
//        mag = 0.577350269f*nB + 0.577350269f*nG + 0.577350269f*nR;
//        if(mag > maxmag) //It's White or Black or in between
//        {
//          maxmag = mag;
          out = 64;
          if(cmag < 3*thresh_bw) out = 128;
//         }
      }
      *o = out;
      if(!(foo%80123)){
        std::cout << "[bgr]= [" << int(*b) << ", " << int(*g) << ", " << int(*r) << "] m= " << maxmag << ", o = " << int(out) << std::endl;
        std::cout << "[nB,nG,nR]: [" << nB << ", " << nG << ", " << nR << "]\n" << std::endl;
      }

#endif

      }
    }
#endif
    cv::Mat Ihsv;
    Ihsv.create(Iin.size(),CV_8UC3);
    cvtColor(Iin, Ihsv, CV_BGR2HSV);
    for (int y = 0; y < Ihsv.rows; y++)
    {
      const uchar *h = Ihsv.ptr<uchar> (y);
      const uchar *s = h + 1;
      const uchar *v = h + 2;
      uchar *o = Iord.ptr<uchar> (y);
      for (int x = 0; x < Ihsv.cols; x++, h += 3, s += 3, v += 3, o++)
      {
        /*
        *             Binary color code
   Bit set        Color values  Category
   =======         =========     ======        B   G   R
     0   1         B>>R or G     "Blue"       [1,  0,  0  ]
     1   2         B~=G >> R     "Turquoise"  [rt2,rt2,0  ]
     2   4         G>>B or R     "Green"      [0,  1,  0  ]
     3   8        G~=R >> B      "Yellow"     [0,  rt2,rt2]
     4  16         R>>G or B     "Red"        [0,  0,   1 ]
     5  32         R~=B >> G     "Purple"     [rt2,0,  rt2]
     6  64         R,G,B >> 0    "White"      [rt3,rt3,rt3]  high mag
     7 128         R,G,B ~= 0    "Black"      [rt3,rt3,rt3]  low mag
         */
        if(*h < 30) *o = 32; //red purple
        else if (*h < 60) *o = 16; //yellow red
        else if (*h < 90) *o = 8; //green yellow
        else if (*h < 120) *o = 4; //Turquoise or cyan
        else if (*h < 150) *o = 2; //blue turquios
        else if (*h <= 180) *o = 1; //purple blue
        if(*v < thresh_bw) *o = 128;
        else if (*v > 255 - thresh_bw) *o = 64;
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

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
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
  void idealize_colors(cv::Mat Iin, cv::Mat& Iout)
  {
    //decode binary images here:
    //     GaussianBlur(Iin, Iord, Size(5, 5), 2, 2);
    if (Iin.size() != Iout.size() || Iout.type() != CV_8UC3) //Make sure Iout is the right size and type
    {
      Iout.create(Iin.size(), CV_8UC3);
    }
    for (int y = 0; y < Iin.rows; y++)
    {
      uchar *b = Iout.ptr<uchar> (y);
      uchar *g = b + 1;
      uchar *r = b + 2;
      uchar *o = Iin.ptr<uchar> (y);
      /**
       *
       *
            Binary color code
   Bit set        Color values  Category
   =======         =========     ======
     0   1         B>>R or G     "Blue"
     1   2         B~=G >> R     "Turquoise"
     2   4         G>>B or R     "Green"
     3   8        G~=R >> B      "Yellow"
     4  16         R>>G or B     "Red"
     5  32         R~=B >> G     "Purple"
     6  64         R,G,B >> 0    "White"
     7 128         R,G,B ~= 0    "Black"
       */
      for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
      {
        if(*o == 0) //No color => gray
        { *b = 100; *r = 100; *g = 100;}
        else if(*o == 1)  //B
        { *b = 255; *r = 0; *g = 0; }
        else if(*o == 2)  //BG Turquois
        { *b = 255; *r = 0; *g = 255; }
        else if(*o == 4)  //G
        { *b = 0; *r = 0; *g = 255; }
        else if(*o == 8)  //GR Yellow
        { *b = 0; *r = 255; *g = 255; }
        else if(*o == 16)  //R
        { *b = 0; *r = 255; *g = 0; }
        else if(*o == 32)  //RB Purple
        { *b = 255; *r = 255; *g = 0; }
        else if(*o == 64)  //White
        { *b = 255; *r = 255; *g = 255; }
        else if(*o == 128) //Black
        { *b = 0; *r = 0; *g = 0; }
      }
  }
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
