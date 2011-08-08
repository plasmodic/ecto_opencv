#include <ecto/ecto.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
using ecto::tendrils;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

struct ImageReader
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("path", "The path to read images from.",
                                 "/tmp/ecto/rules");
    params.declare<std::string> ("ext", "The image extension to look for.",
                                 ".png|.jpg|.bmp");
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs)
  {
    //set outputs
    outputs.declare<cv::Mat> ("out", "A video frame.", cv::Mat());
    outputs.declare<int> ("frame_number", "The number of frames captured.", 0);
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    path = params.get<std::string> ("path");
    ext = params.get<std::string> ("ext");
    fs::path x(path);
    if (!fs::is_directory(x))
      throw std::runtime_error(path + " is not a directory");

    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_itr(path); dir_itr != end_iter; ++dir_itr)
    {
      try
      {
        if (fs::is_regular_file(dir_itr->status()))
        {
          fs::path x(*dir_itr);
          if (x.extension().size() == 0 || ext.find(x.extension())
              == std::string::npos)
            continue;
          //std::cout << x.string() << "\n";
          images.push_back(x.string());
        }
      } catch (const std::exception &)
      {
        //std::cout << dir_itr->filename() << " " << ex.what() << std::endl;
      }
    }
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    if (images.empty())
      return 1;
    //outputs.get is a reference;
    outputs.get<cv::Mat> ("out") = cv::imread(images.back());
    images.pop_back();
    //increment our frame number.
    ++(outputs.get<int> ("frame_number"));
    return 0;
  }
  std::string path;
  std::string ext;
  std::vector<std::string> images;

};

struct imread
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("image_file", "The path to the image to read.",
                                 "lena.jpg");
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs)
  {
    //set outputs
    outputs.declare<cv::Mat> ("image", "The image in full color.", cv::Mat());
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    std::string file;
    params["image_file"] >> file;
    cv::Mat image = cv::imread(file);
    outputs["image"] << image;
  }

};
void declare_video_device_outputs(tendrils& outputs)
{
  //set outputs
  outputs.declare<cv::Mat> ("image", "A video frame.", cv::Mat());
  outputs.declare<int> ("frame_number", "The number of frames captured.", 0);
}

struct OpenNICapture
{
  static void declare_params(tendrils& params)
  {
    params.declare<int> ("video_mode", "Video size mode",
                         CV_CAP_OPENNI_VGA_30HZ);

  }

  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs)
  {
    //set outputs
    outputs.declare<cv::Mat> ("depth", "The output depth map", cv::Mat());
    outputs.declare<cv::Mat> ("valid", "The output valid mask", cv::Mat());

    declare_video_device_outputs(outputs);
    outputs.declare<cv::Mat> ("K", "The camera intrinsic matrix.");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    int mode = params.get<int> ("video_mode");
    capture = cv::VideoCapture(CV_CAP_OPENNI);
    capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, mode);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION_ON, true);
    // Print some avalible Kinect settings.
    std::cout << "\nDepth generator output mode:" << std::endl
        << "FRAME_WIDTH    " << capture.get(CV_CAP_PROP_FRAME_WIDTH)
        << std::endl << "FRAME_HEIGHT   "
        << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl
        << "FRAME_MAX_DEPTH    "
        << capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm"
        << std::endl << "FPS    " << capture.get(CV_CAP_PROP_FPS) << std::endl;

    double frame_width = capture.get(
                                     CV_CAP_OPENNI_IMAGE_GENERATOR
                                         + CV_CAP_PROP_FRAME_WIDTH);
    double frame_height = capture.get(
                                      CV_CAP_OPENNI_IMAGE_GENERATOR
                                          + CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "\nImage generator output mode:" << std::endl
        << "FRAME_WIDTH    " << frame_width << std::endl << "FRAME_HEIGHT   "
        << frame_height << std::endl << "FPS    "
        << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FPS)
        << std::endl;
    double focal_length =
        capture.get(CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
    // Simple camera matrix: square pixels, principal point at center
    K.create(3, 3, CV_64F);
    K = cv::Scalar::all(0);
    K.at<double> (0, 0) = K.at<double> (1, 1) = focal_length;
    K.at<double> (0, 2) = (frame_width / 2) - 0.5;
    K.at<double> (1, 2) = (frame_width * 3. / 8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
    K.at<double> (2, 2) = 1.0;
    std::cout << K << std::endl;
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    if (!capture.grab())
    {
      std::cout << "Can not grab images." << std::endl;
      return 1;
    }
    //outputs.get is a reference;
    capture.retrieve(outputs.get<cv::Mat> ("depth"), CV_CAP_OPENNI_DEPTH_MAP);
    capture.retrieve(outputs.get<cv::Mat> ("image"), CV_CAP_OPENNI_BGR_IMAGE);
    capture.retrieve(outputs.get<cv::Mat> ("valid"),
                     CV_CAP_OPENNI_VALID_DEPTH_MASK);
    outputs.get<cv::Mat> ("K") = K;
    //increment our frame number.
    ++(outputs.get<int> ("frame_number"));
    return 0;
  }

  cv::VideoCapture capture;
  cv::Mat K;
};

struct VideoCapture
{
  static void declare_params(tendrils& params)
  {
    params.declare<int> ("video_device", "The device ID to open.", 0);
    params.declare<std::string> (
                                 "video_file",
                                 "A video file to read, leave empty to open a video device.",
                                 "");
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs)
  {
    //set outputs
    declare_video_device_outputs(outputs);
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    video_device = params.get<int> ("video_device");
    video_file = params.get<std::string> ("video_file");
    capture = cv::VideoCapture();
  }
  void open_video_device()
  {
    if (capture.isOpened())
      return;

    if (!video_file.empty())
    {
      capture.open(video_file);
      if (!capture.isOpened())
        throw std::runtime_error(
                                 "Could not open the video file : "
                                     + video_file);
    }
    else
    {
      capture.open(video_device);
      if (!capture.isOpened())
        throw std::runtime_error(
                                 "Could not open video device : "
                                     + video_device);
    }
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    open_video_device();
    cv::Mat image;
    //outputs.get is a reference;
    capture >> image;
    outputs["image"] << image;

    //increment our frame number.
    ++(outputs.get<int> ("frame_number"));
    return 0;
  }
  cv::VideoCapture capture;
  int video_device;
  std::string video_file;

};

struct imshow
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("name", "The window name", "image");
    params.declare<int> (
                         "waitKey",
                         "Number of millis to wait, -1 for not at all, 0 for infinity.",
                         -1);
    params.declare<bool> ("autoSize", "Autosize the window.", true);
    params.declare<bool> ("maximize", "Fullscreen the window, takes precedence over autoSize.", false);
  }

  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs)
  {
    inputs.declare<cv::Mat> ("input", "The image to show");
    outputs.declare<int> ("out", "Character pressed.");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    window_name_ = params.get<std::string> ("name");
    waitkey_ = params.get<int> ("waitKey");
    auto_size_ = params.get<bool> ("autoSize");
    full_screen_ = params["maximize"];
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("input");
    if (image.empty())
    {
      outputs.get<int> ("out") = 0;
      return 0;
      //throw std::logic_error("empty image!");
    }
    if(*full_screen_)
    {
      cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
      cv::setWindowProperty(window_name_,CV_WND_PROP_FULLSCREEN,true);
    }
    else if (auto_size_)
    {
      cv::namedWindow(window_name_, CV_WINDOW_KEEPRATIO);
    }

    if (image.depth() == CV_32F || image.depth() == CV_64F)
    {
      const float scaleFactor = 100;
      cv::Mat show;
      image.convertTo(show, CV_8UC1, scaleFactor);
      image = show;
    }

    if (image.type() == CV_16UC1)
    {
      const float scaleFactor = 0.05f;
      cv::Mat show;
      image.convertTo(show, CV_8UC1, scaleFactor);
      image = show;
    }

    cv::imshow(window_name_, image);

    int r = 0;
    if (waitkey_ >= 0) 
      r = 0xff & cv::waitKey(waitkey_);

    if (r == 27 || r == 'q' || r == 'Q') 
    {
      std::cout << "QUIT!\n";
      return 1;
    }
    else
    {
      outputs.get<int> ("out") = r;
      return 0;
    }
  }
  std::string window_name_;
  int waitkey_;
  bool auto_size_;
  ecto::spore<bool> full_screen_;
};
ECTO_THREAD_UNSAFE(imshow);


struct FPSDrawer
{
  static void draw(cv::Mat& drawImage, float freq)
  {
    using namespace cv;
    std::string scaleText = boost::str(boost::format("%.2f Hz")%freq);
    int baseline = 0;
    Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
    rectangle(drawImage, Point(10, 30 + 5),
              Point(10, 30) + Point(sz.width, -sz.height - 5), Scalar::all(0),
              -1);
    putText(drawImage, scaleText, Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 1.0,
            Scalar::all(255), 1, CV_AA, false);
  }
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("image", "The original image to draw the pose onto.");
    out.declare<cv::Mat> ("image",
                          "The image with fps drawn on it.");
  }
  FPSDrawer() :count(),freq(){}
  int process(const tendrils& in, tendrils& out)
  {
    pt::ptime now = pt::microsec_clock::universal_time();

    if(count == 0 )
    {
      prev = now;
    }else if(count == 30)
    {
      pt::time_duration elapsed = now - prev;
      freq = double(count)/(elapsed.total_microseconds() * 1e-6);
      prev = now;
      count = 0;
    }
    count++;
    cv::Mat image = in.get<cv::Mat> ("image");
    cv::Mat& output = out.get<cv::Mat> ("image");
    image.copyTo(output);
    draw(output, freq);
    return 0;
  }
  pt::ptime prev;
  size_t count;
  double freq;
};

struct ImageSaver
{
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("filename", "The filename prefix to save the image to.", "./image_");

  }
  ImageSaver():count(0){}
  static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
  {
    in.declare<cv::Mat> ("image", "The original image to draw the pose onto.");
    in.declare<int> (
                             "trigger",
                             "'s' to save.",
                             0);
  }
  void configure(tendrils&p,tendrils&in,tendrils&o)
  {
    prefix = p["filename"];
  }
  int process(tendrils& in, tendrils& out)
  {
    int trigger;
    in["trigger"] >> trigger;
    if(trigger != 's'){
      return 0;
    }
    cv::Mat image;
    in["image"] >> image;
    std::string filename = boost::str(boost::format("%s%04d.png")%*prefix%count++);
    std::cout << "Saving image to : " << filename << std::endl;
    cv::imwrite(filename,image);
    return 0;
  }
  int count;
  ecto::spore<std::string> prefix;
};

ECTO_DEFINE_MODULE(highgui)
{
  ecto::wrap<VideoCapture>("VideoCapture",
                           "Use to capture video from a camera or video file.");
  ecto::wrap<imshow>("imshow", "Shows an image in a named window.");
  ecto::wrap<ImageReader>("ImageReader", "Read images from a directory.");
  ecto::wrap<OpenNICapture>("OpenNICapture", "OpenNI capture device.");
  ecto::wrap<FPSDrawer>("FPSDrawer", "Draw the Hz on an image.");
  ecto::wrap<ImageSaver>("ImageSaver","A png file saver for images.");
  ecto::wrap<imread>("imread","A single png reader.");


}
