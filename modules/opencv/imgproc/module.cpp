#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
namespace bp = boost::python;

ECTO_DEFINE_MODULE(imgproc)
{
  bp::object proc = bp::scope();
  /* Constants for color conversion */
  proc.attr("CV_BGR2BGRA") = int(CV_BGR2BGRA);
  proc.attr("CV_RGB2RGBA") = int(CV_RGB2RGBA);

  proc.attr("CV_BGRA2BGR") = int(CV_BGRA2BGR);
  proc.attr("CV_RGBA2RGB") = int(CV_RGBA2RGB);

  proc.attr("CV_BGR2RGBA") = int(CV_BGR2RGBA);
  proc.attr("CV_RGB2BGRA") = int(CV_RGB2BGRA);

  proc.attr("CV_RGBA2BGR") = int(CV_RGBA2BGR);
  proc.attr("CV_BGRA2RGB") = int(CV_BGRA2RGB);

  proc.attr("CV_BGR2RGB") = int(CV_BGR2RGB);
  proc.attr("CV_RGB2BGR") = int(CV_RGB2BGR);

  proc.attr("CV_BGRA2RGBA") = int(CV_BGRA2RGBA);
  proc.attr("CV_RGBA2BGRA") = int(CV_RGBA2BGRA);

  proc.attr("CV_BGR2GRAY") = int(CV_BGR2GRAY);
  proc.attr("CV_RGB2GRAY") = int(CV_RGB2GRAY);
  proc.attr("CV_GRAY2BGR") = int(CV_GRAY2BGR);
  proc.attr("CV_GRAY2RGB") = int(CV_GRAY2RGB);
  proc.attr("CV_GRAY2BGRA") = int(CV_GRAY2BGRA);
  proc.attr("CV_GRAY2RGBA") = int(CV_GRAY2RGBA);
  proc.attr("CV_BGRA2GRAY") = int(CV_BGRA2GRAY);
  proc.attr("CV_RGBA2GRAY") = int(CV_RGBA2GRAY);

  proc.attr("CV_BGR2BGR565") = int(CV_BGR2BGR565);
  proc.attr("CV_RGB2BGR565") = int(CV_RGB2BGR565);
  proc.attr("CV_BGR5652BGR") = int(CV_BGR5652BGR);
  proc.attr("CV_BGR5652RGB") = int(CV_BGR5652RGB);
  proc.attr("CV_BGRA2BGR565") = int(CV_BGRA2BGR565);
  proc.attr("CV_RGBA2BGR565") = int(CV_RGBA2BGR565);
  proc.attr("CV_BGR5652BGRA") = int(CV_BGR5652BGRA);
  proc.attr("CV_BGR5652RGBA") = int(CV_BGR5652RGBA);

  proc.attr("CV_GRAY2BGR565") = int(CV_GRAY2BGR565);
  proc.attr("CV_BGR5652GRAY") = int(CV_BGR5652GRAY);

  proc.attr("CV_BGR2BGR555") = int(CV_BGR2BGR555);
  proc.attr("CV_RGB2BGR555") = int(CV_RGB2BGR555);
  proc.attr("CV_BGR5552BGR") = int(CV_BGR5552BGR);
  proc.attr("CV_BGR5552RGB") = int(CV_BGR5552RGB);
  proc.attr("CV_BGRA2BGR555") = int(CV_BGRA2BGR555);
  proc.attr("CV_RGBA2BGR555") = int(CV_RGBA2BGR555);
  proc.attr("CV_BGR5552BGRA") = int(CV_BGR5552BGRA);
  proc.attr("CV_BGR5552RGBA") = int(CV_BGR5552RGBA);

  proc.attr("CV_GRAY2BGR555") = int(CV_GRAY2BGR555);
  proc.attr("CV_BGR5552GRAY") = int(CV_BGR5552GRAY);

  proc.attr("CV_BGR2XYZ") = int(CV_BGR2XYZ);
  proc.attr("CV_RGB2XYZ") = int(CV_RGB2XYZ);
  proc.attr("CV_XYZ2BGR") = int(CV_XYZ2BGR);
  proc.attr("CV_XYZ2RGB") = int(CV_XYZ2RGB);

  proc.attr("CV_BGR2YCrCb") = int(CV_BGR2YCrCb);
  proc.attr("CV_RGB2YCrCb") = int(CV_RGB2YCrCb);
  proc.attr("CV_YCrCb2BGR") = int(CV_YCrCb2BGR);
  proc.attr("CV_YCrCb2RGB") = int(CV_YCrCb2RGB);

  proc.attr("CV_BGR2HSV") = int(CV_BGR2HSV);
  proc.attr("CV_RGB2HSV") = int(CV_RGB2HSV);

  proc.attr("CV_BGR2Lab") = int(CV_BGR2Lab);
  proc.attr("CV_RGB2Lab") = int(CV_RGB2Lab);

  proc.attr("CV_BayerBG2BGR") = int(CV_BayerBG2BGR);
  proc.attr("CV_BayerGB2BGR") = int(CV_BayerGB2BGR);
  proc.attr("CV_BayerRG2BGR") = int(CV_BayerRG2BGR);
  proc.attr("CV_BayerGR2BGR") = int(CV_BayerGR2BGR);

  proc.attr("CV_BayerBG2RGB") = int(CV_BayerBG2RGB);
  proc.attr("CV_BayerGB2RGB") = int(CV_BayerGB2RGB);
  proc.attr("CV_BayerRG2RGB") = int(CV_BayerRG2RGB);
  proc.attr("CV_BayerGR2RGB") = int(CV_BayerGR2RGB);

  proc.attr("CV_BGR2Luv") = int(CV_BGR2Luv);
  proc.attr("CV_RGB2Luv") = int(CV_RGB2Luv);
  proc.attr("CV_BGR2HLS") = int(CV_BGR2HLS);
  proc.attr("CV_RGB2HLS") = int(CV_RGB2HLS);

  proc.attr("CV_HSV2BGR") = int(CV_HSV2BGR);
  proc.attr("CV_HSV2RGB") = int(CV_HSV2RGB);

  proc.attr("CV_Lab2BGR") = int(CV_Lab2BGR);
  proc.attr("CV_Lab2RGB") = int(CV_Lab2RGB);
  proc.attr("CV_Luv2BGR") = int(CV_Luv2BGR);
  proc.attr("CV_Luv2RGB") = int(CV_Luv2RGB);
  proc.attr("CV_HLS2BGR") = int(CV_HLS2BGR);
  proc.attr("CV_HLS2RGB") = int(CV_HLS2RGB);

  proc.attr("CV_BayerBG2BGR_VNG") = int(CV_BayerBG2BGR_VNG);
  proc.attr("CV_BayerGB2BGR_VNG") = int(CV_BayerGB2BGR_VNG);
  proc.attr("CV_BayerRG2BGR_VNG") = int(CV_BayerRG2BGR_VNG);
  proc.attr("CV_BayerGR2BGR_VNG") = int(CV_BayerGR2BGR_VNG);

  proc.attr("CV_BayerBG2RGB_VNG") = int(CV_BayerBG2RGB_VNG);
  proc.attr("CV_BayerGB2RGB_VNG") = int(CV_BayerGB2RGB_VNG);
  proc.attr("CV_BayerRG2RGB_VNG") = int(CV_BayerRG2RGB_VNG);
  proc.attr("CV_BayerGR2RGB_VNG") = int(CV_BayerGR2RGB_VNG);

  proc.attr("CV_BGR2HSV_FULL") = int(CV_BGR2HSV_FULL);
  proc.attr("CV_RGB2HSV_FULL") = int(CV_RGB2HSV_FULL);
  proc.attr("CV_BGR2HLS_FULL") = int(CV_BGR2HLS_FULL);
  proc.attr("CV_RGB2HLS_FULL") = int(CV_RGB2HLS_FULL);

  proc.attr("CV_HSV2BGR_FULL") = int(CV_HSV2BGR_FULL);
  proc.attr("CV_HSV2RGB_FULL") = int(CV_HSV2RGB_FULL);
  proc.attr("CV_HLS2BGR_FULL") = int(CV_HLS2BGR_FULL);
  proc.attr("CV_HLS2RGB_FULL") = int(CV_HLS2RGB_FULL);

  proc.attr("CV_LBGR2Lab") = int(CV_LBGR2Lab);
  proc.attr("CV_LRGB2Lab") = int(CV_LRGB2Lab);
  proc.attr("CV_LBGR2Luv") = int(CV_LBGR2Luv);
  proc.attr("CV_LRGB2Luv") = int(CV_LRGB2Luv);

  proc.attr("CV_Lab2LBGR") = int(CV_Lab2LBGR);
  proc.attr("CV_Lab2LRGB") = int(CV_Lab2LRGB);
  proc.attr("CV_Luv2LBGR") = int(CV_Luv2LBGR);
  proc.attr("CV_Luv2LRGB") = int(CV_Luv2LRGB);

  proc.attr("CV_BGR2YUV") = int(CV_BGR2YUV);
  proc.attr("CV_RGB2YUV") = int(CV_RGB2YUV);
  proc.attr("CV_YUV2BGR") = int(CV_YUV2BGR);
  proc.attr("CV_YUV2RGB") = int(CV_YUV2RGB);

  proc.attr("CV_BayerBG2GRAY") = int(CV_BayerBG2GRAY);
  proc.attr("CV_BayerGB2GRAY") = int(CV_BayerGB2GRAY);
  proc.attr("CV_BayerRG2GRAY") = int(CV_BayerRG2GRAY);
  proc.attr("CV_BayerGR2GRAY") = int(CV_BayerGR2GRAY);

  proc.attr("CV_YUV420i2RGB") = int(CV_YUV420i2RGB);
  proc.attr("CV_YUV420i2BGR") = int(CV_YUV420i2BGR);
}
