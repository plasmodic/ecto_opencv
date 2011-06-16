#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

using ecto::tendrils;
using namespace cv;
using namespace std;

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
  void computeColorOrder(const cv::Mat &Iina, cv::Mat &Icolorord)
  {
//    Mat Iin;
//    resize(Iin,Iin,Size(Iin.cols/8,Iin.rows/8));
    Mat Iin;
      GaussianBlur(Iina, Iin, Size(gsize, gsize), gsig, gsig);

//    Mat Ip2,Ip4,Iin;
//    pyrDown(Iina,Ip2);
//    pyrDown(Ip2,Ip4);
//    pyrDown(Ip4,Iin);
    if (Iin.size() != Icolorord.size() || Icolorord.type() != CV_8UC1) //Make sure Icolorord is the right size
    {
      Icolorord.create(Iin.size(), CV_8UC1);
    }
    //float Bu = 0, Bs = 0, Gu = 0, Gs = 0, Ru = 0, Rs = 0, BM = 0, Bm = 1000, GM = 0, Gm = 1000, RM = 0, Rm = 1000;
    float cratio;
    uchar oresult;
 //   int foo = 1;
    for (int y = 0; y < Iin.rows; y++)
    {
      const uchar *b = Iin.ptr<uchar> (y);
      const uchar *g = b + 1;
      const uchar *r = b + 2;
      uchar *o = Icolorord.ptr<uchar> (y);
      for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
      {
        /**
         * 2 bits each: 0=><64, 1=><128, 2=><192 3 >= 192
         * B/G R/B G/R
         */
        float B = float(unsigned(*b))+1.0, R = float(unsigned(*r))+1.0, G = float(unsigned(*g))+1.0; //protect from 0

        oresult = 0;
        // B/G
        cratio = B/G;
//        Bu += cratio;
//        Bs += cratio*cratio;
//        if(Bm > cratio) Bm = cratio;
//        if(BM < cratio) BM = cratio;
        if(cratio > 1.25)
        { oresult |= 3;}
        else if(cratio > 1.0)
          oresult |= 2;
        else if(cratio > 0.75)
          oresult |=1;
        // R/B
        cratio = R/B;
//        Ru += cratio;
//        Rs += cratio*cratio;
//        if(Rm > cratio) Rm = cratio;
//        if(RM < cratio) RM = cratio;
        if(cratio > 1.25)
        { oresult |= 12;}
        else if(cratio > 1.0)
          oresult |= 8;
        else if(cratio > 0.75)
          oresult |=4;
        // G/R
        cratio = G/R;
//        Gu += cratio;
//        Gs += cratio*cratio;
//        if(Gm > cratio) Gm = cratio;
//        if(GM < cratio) GM = cratio;
        if(cratio > 1.25)
        { oresult |= 48;}
        else if(cratio > 1.0)
          oresult |= 32;
        else if(cratio > 0.75)
          oresult |=16;
        //Determine black/white status
        if((R>192)&&(G>192)&&(B>192))
          oresult |= 128; //10______ means white
        else if((64>R)&&(64>G)&&(64>B)); //00______ means dark
        else
          oresult |= 64;  //01______ means in between,
//        if(!(foo%30000))
//        {
//          cout << "foo = " << foo << " Bu " << Bu << endl;
//          Bu /= (float)foo;
//          Ru /= (float)foo;
//          Gu /= (float)foo;
//          Bs /= (float)foo;
//          Rs /= (float)foo;
//          Gs /= (float)foo;
//          cout << "Bu=" << Bu << " Bs=" << sqrt(Bs - Bu*Bu) << " BM=" << BM << " Bm=" << Bm << endl;
//          cout << "Gu=" << Gu << " Gs=" << sqrt(Gs - Gu*Gu) << " GM=" << GM << " Gm=" << Gm << endl;
//          cout << "Ru=" << Ru << " Rs=" << sqrt(Rs - Ru*Ru) << " RM=" << RM << " Rm=" << Rm << endl;
//          foo = 1;
//          Bu = 0, Bs = 0, Gu = 0, Gs = 0, Ru = 0, Rs = 0, BM = 0, Bm = 1000, GM = 0, Gm = 1000, RM = 0, Rm = 1000;
//       }
//        foo++;

        *o = oresult;
      }
    }
  }

  static void declare_params(tendrils& p)
  {
    p.declare<int> ("gsize", "Size of NxN Gaussian blur template.", 5);
    p.declare<double> ("gsig","Sigma of Gaussian blur template", 2.0);
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("image", "An input image. RGB");
    outputs.declare<cv::Mat> ("output", "A binarized color image.");
  }

  void configure(tendrils& params)
  {
    gsize = params.get<int> ("gsize");
    gsig = params.get<double> ("gsig");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("image");
    cv::Mat& output = outputs.get<cv::Mat> ("output");
    computeColorOrder(image, output);
    return 0;
  }

  int gsize;   //size of NxN Gaussian blur template
  double gsig; //Size of Gaussian blur Sigma

};

/////////////////////////////////////////////////////////////
static int response_table[4][4] =
{ {3,2,1,0},
  {2,3,2,1},
  {1,2,3,2},
  {0,1,2,3}};
struct ColorTempl
{
  //Storage
  std::vector<Point> offsets; //The template starts from the upper left corner of the bounding box of its mask
  std::vector<uchar> coded_color;
  Size template_size;
  /**
   * double response(const Mat image, const Point offset);
   * @param image
   * @param offset
   * @return 1.0 = perfect match, 0 not.
   */
  double response(const Mat image, const Point offset)
  {
    if(image.type != CV_8UC1)
    {
      cout << "ERROR: Image is not of type CV_8UC1 in ColorTempl" << endl;
      return;
    }
    if(image.channels() != 1)
    {
      cout << "ERROR, image in ColorTempl is not one channel" << endl;
    }
    int len = (int)offset.size();
    int step = image.step;
    const uchar *base = image.ptr<uchar> (offset.y);
    base += offset.x;
    int score = 0;
    int t,m; //test and model
    for(int i = 0; i<len; ++i)
    {
      uchar test = *(base + (step*offsets[i].y) + offsets[i].x);
      //B/G
      t = (int)(text & 0x3);
      m = (int)(coded_color[i] & 0x3);
      score += response_table[t][m];
      //R/B
      t = (int)((text>>2)& 0x3);
      m = (int)((coded_color[i]>>2) & Ox3);
      score += response_table[t][m];
      //G/R
      t = (int)((text>>4)& 0x3);
      m = (int)((coded_color[i]>>4) & Ox3);
      score += response_table[t][m];
      //Black and white
      t = (int)((text>>6)& 0x3);
      m = (int)((coded_color[i]>>6) & Ox3);
      score += response_table[t][m] - 1; // minus one because black and white can differ only by 2 not 3
    }
    return (double)score/(double)(len*11);
  }

  void clear()
  {
    offsets.clear();
    coded_color.clear();
    template_size = Size(0,0);
  }

};

/**
 * Calculate a color template
 * Note that the template starts from the upper left corner of the bounding box of the mask
 */
struct ColorTemplCalc
{
  void learn_a_template(const Mat &Icolorord, const Mat &Mask, ColorTempl &ct )
  {
    ct.clear();
    if(Icolorord.size() != Mask.size())
    {
      cout << "ERROR Icolorord and Mask were not the same size in ColorTemplCalc" << endl;
      return;
    }
    if((Icolorord.channels() ! = Mask.channels())&&(Mask.channels() != 1))
    {
      cout << "ERROR: channels missmatch or != 1 in ColorTemplCalc" << endl;
      return;
    }
    if((Icolorord.type != CV_8UC1) || (Mask.type != CV_8UC1))
    {
      cout << "ERROR: Icolorord and/or Mask is not of type CV_8UC1 in ColorTemplCalc" << endl;
      return;
    }
    //Find upperleft corner of mask
    int cY = 0,cYend, minx = Mask.cols;
    vector<int> cX;
    for(int y = 0; y < Mask.rows; y++)
    {
      const uchar *m = Mask.ptr<uchar> (y);
      for(int x = 0; x<Mask.cols; x++, m++)
      {
        if(*m)
        {
          if(!cY)
            cY = y; //Mark the upper left corner, we'll need it for offsets below
          cYend = y;
          cX.push_back(x);
          if(minx > x)
            minx = x;
          break;
        }
      }
    }

    //Log the template
    int Maxx = minx;
    int Maxy = 0;
    for (int y = cY; y < cYend; y+=skipy)
    {
      const uchar *m = Mask.ptr<uchar> (y);
      const uchar *o = Icolorord.ptr<uchar> (y);
      for (int x = cX[y-cY]; x < Icolorord.cols; x+=skipx, m+=skipx, o+=skipx)
      {
        if(*m)
        {
          ct.coded_color.push_back(*o);
          ct.offsets.push_back(Point(x-minX,y-cY));
          if(Maxx < x)
            Maxx = x;
          if(Maxy < y)
            Maxy = y;
        }
      }
    }
    ct.Size = Size(Maxx - minx, Maxy - cY); //bounding box of the actual touched points in the mask
  }

  //Virtual functs
  static void declare_params(tendrils& p)
  {
    p.declare<int> ("skipx", "Skip every skipx point in the x direction when collecting a template.", 5);
    p.declare<int> ("skipy","Skip every skipy point in the y direction when collecting a template.t", 5);
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
      tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("image", "A color coded template from the ColorMod module");
    inputs.declare<cv::Mat> ("mask", "A mask of the object");
    outputs.declare<ColorTempl> ("output", "A color template of type ColorTempl.");
  }

  void configure(tendrils& params)
  {
    skipx = params.get<int> ("skipx");
    skipy = params.get<int> ("skipy");
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat &image = inputs.get<cv::Mat> ("image");
    cv::Mat &mask = inputs.get<cv::Mat> ("mask");
    ColorTempl &output = outputs.get<ColorTempl> ("ColorTempl");
    learn_a_template(image, mask, ColorTempl);
    return 0;
  }
  //settable
  int skipx, skipy;   //How many pixels to skip in the X and Y directions when collecting a template



};



struct ColorDebug
{
  void idealize_colors(cv::Mat Iin, cv::Mat& Iout)
  {
    //decode binary images here:
         GaussianBlur(Iin, Iin, Size(5, 5), 2, 2);
    if (Iin.size() != Iout.size() || Iout.type() != CV_8UC3) //Make sure Iout is the right size and type
    {
      Iout.create(Iin.size(), CV_8UC3);
    }
    uchar cratio;
    int foo = 0;
    for (int y = 0; y < Iin.rows; y++)
    {
      uchar *b = Iout.ptr<uchar> (y);
      uchar *g = b + 1;
      uchar *r = b + 2;
      uchar *o = Iin.ptr<uchar> (y);
      for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
      {
        /**
         * 2 bits each: 0=><64, 1=><128, 2=><192 3 >= 192
         * B/G R/B G/R
         */
        *b = 0; *r = 0; *g = 0;
        //B/G
        cratio = *o & 3;
//        if(!(foo%70000))
//          cout << "B/G (3-0)= " << (unsigned)cratio << endl;
        if(cratio == 3)
          *b = 255;
        else if(cratio == 2)
          *b = 196;
        else if (cratio == 1)
          *b = 128;
        //R/B
        cratio = *o & 12;
//        if(!(foo%70000))
//          cout << "R/G (12-4)= " << (unsigned)cratio << endl;
        if(cratio == 12)
          *r = 255;
        else if(cratio == 8)
          *r = 196;
        else if (cratio == 4)
          *r = 128;
        //G/R
        cratio = *o & 48;
//        if(!(foo%70000))
//          cout << "G/R (48-16)= " << (unsigned)cratio << endl;
        if(cratio == 48)
          *g = 255;
        else if(cratio == 32)
          *g = 196;
        else if (cratio == 16)
          *g = 128;
        //B&W
        cratio = *o & 192;
        if(cratio == 128)
        {
          if(!(foo%5)){
          *g = 255; *r = 255; *b = 255;}
          foo++;
        } else if (cratio == 0)
        {
          if(!(foo%5)){
          *g = 0; *r = 0; *b = 0;}
          foo++;
        }

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
