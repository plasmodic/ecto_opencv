#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
namespace bp = boost::python;

ECTO_DEFINE_MODULE(imgproc)
{
  using namespace imgproc;
  bp::object proc = bp::scope();
  bp::enum_<Conversion> eConversion("Conversion");
  eConversion.value("BGR2BGRA", BGR2BGRA);
  eConversion.value("BGR2BGRA", BGR2BGRA);
  eConversion.value("RGB2RGBA", RGB2RGBA);

  eConversion.value("BGRA2BGR", BGRA2BGR);
  eConversion.value("RGBA2RGB", RGBA2RGB);

  eConversion.value("BGR2RGBA", BGR2RGBA);
  eConversion.value("RGB2BGRA", RGB2BGRA);

  eConversion.value("RGBA2BGR", RGBA2BGR);
  eConversion.value("BGRA2RGB", BGRA2RGB);

  eConversion.value("BGR2RGB", BGR2RGB);
  eConversion.value("RGB2BGR", RGB2BGR);

  eConversion.value("BGRA2RGBA", BGRA2RGBA);
  eConversion.value("RGBA2BGRA", RGBA2BGRA);

  eConversion.value("BGR2GRAY", BGR2GRAY);
  eConversion.value("RGB2GRAY", RGB2GRAY);
  eConversion.value("GRAY2BGR", GRAY2BGR);
  eConversion.value("GRAY2RGB", GRAY2RGB);
  eConversion.value("GRAY2BGRA", GRAY2BGRA);
  eConversion.value("GRAY2RGBA", GRAY2RGBA);
  eConversion.value("BGRA2GRAY", BGRA2GRAY);
  eConversion.value("RGBA2GRAY", RGBA2GRAY);

  eConversion.value("BGR2BGR565", BGR2BGR565);
  eConversion.value("RGB2BGR565", RGB2BGR565);
  eConversion.value("BGR5652BGR", BGR5652BGR);
  eConversion.value("BGR5652RGB", BGR5652RGB);
  eConversion.value("BGRA2BGR565", BGRA2BGR565);
  eConversion.value("RGBA2BGR565", RGBA2BGR565);
  eConversion.value("BGR5652BGRA", BGR5652BGRA);
  eConversion.value("BGR5652RGBA", BGR5652RGBA);

  eConversion.value("GRAY2BGR565", GRAY2BGR565);
  eConversion.value("BGR5652GRAY", BGR5652GRAY);

  eConversion.value("BGR2BGR555", BGR2BGR555);
  eConversion.value("RGB2BGR555", RGB2BGR555);
  eConversion.value("BGR5552BGR", BGR5552BGR);
  eConversion.value("BGR5552RGB", BGR5552RGB);
  eConversion.value("BGRA2BGR555", BGRA2BGR555);
  eConversion.value("RGBA2BGR555", RGBA2BGR555);
  eConversion.value("BGR5552BGRA", BGR5552BGRA);
  eConversion.value("BGR5552RGBA", BGR5552RGBA);

  eConversion.value("GRAY2BGR555", GRAY2BGR555);
  eConversion.value("BGR5552GRAY", BGR5552GRAY);

  eConversion.value("BGR2XYZ", BGR2XYZ);
  eConversion.value("RGB2XYZ", RGB2XYZ);
  eConversion.value("XYZ2BGR", XYZ2BGR);
  eConversion.value("XYZ2RGB", XYZ2RGB);

  eConversion.value("BGR2YCrCb", BGR2YCrCb);
  eConversion.value("RGB2YCrCb", RGB2YCrCb);
  eConversion.value("YCrCb2BGR", YCrCb2BGR);
  eConversion.value("YCrCb2RGB", YCrCb2RGB);

  eConversion.value("BGR2HSV", BGR2HSV);
  eConversion.value("RGB2HSV", RGB2HSV);

  eConversion.value("BGR2Lab", BGR2Lab);
  eConversion.value("RGB2Lab", RGB2Lab);

  eConversion.value("BayerBG2BGR", BayerBG2BGR);
  eConversion.value("BayerGB2BGR", BayerGB2BGR);
  eConversion.value("BayerRG2BGR", BayerRG2BGR);
  eConversion.value("BayerGR2BGR", BayerGR2BGR);

  eConversion.value("BayerBG2RGB", BayerBG2RGB);
  eConversion.value("BayerGB2RGB", BayerGB2RGB);
  eConversion.value("BayerRG2RGB", BayerRG2RGB);
  eConversion.value("BayerGR2RGB", BayerGR2RGB);

  eConversion.value("BGR2Luv", BGR2Luv);
  eConversion.value("RGB2Luv", RGB2Luv);
  eConversion.value("BGR2HLS", BGR2HLS);
  eConversion.value("RGB2HLS", RGB2HLS);

  eConversion.value("HSV2BGR", HSV2BGR);
  eConversion.value("HSV2RGB", HSV2RGB);

  eConversion.value("Lab2BGR", Lab2BGR);
  eConversion.value("Lab2RGB", Lab2RGB);
  eConversion.value("Luv2BGR", Luv2BGR);
  eConversion.value("Luv2RGB", Luv2RGB);
  eConversion.value("HLS2BGR", HLS2BGR);
  eConversion.value("HLS2RGB", HLS2RGB);

  eConversion.value("BayerBG2BGR_VNG", BayerBG2BGR_VNG);
  eConversion.value("BayerGB2BGR_VNG", BayerGB2BGR_VNG);
  eConversion.value("BayerRG2BGR_VNG", BayerRG2BGR_VNG);
  eConversion.value("BayerGR2BGR_VNG", BayerGR2BGR_VNG);

  eConversion.value("BayerBG2RGB_VNG", BayerBG2RGB_VNG);
  eConversion.value("BayerGB2RGB_VNG", BayerGB2RGB_VNG);
  eConversion.value("BayerRG2RGB_VNG", BayerRG2RGB_VNG);
  eConversion.value("BayerGR2RGB_VNG", BayerGR2RGB_VNG);

  eConversion.value("BGR2HSV_FULL", BGR2HSV_FULL);
  eConversion.value("RGB2HSV_FULL", RGB2HSV_FULL);
  eConversion.value("BGR2HLS_FULL", BGR2HLS_FULL);
  eConversion.value("RGB2HLS_FULL", RGB2HLS_FULL);

  eConversion.value("HSV2BGR_FULL", HSV2BGR_FULL);
  eConversion.value("HSV2RGB_FULL", HSV2RGB_FULL);
  eConversion.value("HLS2BGR_FULL", HLS2BGR_FULL);
  eConversion.value("HLS2RGB_FULL", HLS2RGB_FULL);

  eConversion.value("LBGR2Lab", LBGR2Lab);
  eConversion.value("LRGB2Lab", LRGB2Lab);
  eConversion.value("LBGR2Luv", LBGR2Luv);
  eConversion.value("LRGB2Luv", LRGB2Luv);

  eConversion.value("Lab2LBGR", Lab2LBGR);
  eConversion.value("Lab2LRGB", Lab2LRGB);
  eConversion.value("Luv2LBGR", Luv2LBGR);
  eConversion.value("Luv2LRGB", Luv2LRGB);

  eConversion.value("BGR2YUV", BGR2YUV);
  eConversion.value("RGB2YUV", RGB2YUV);
  eConversion.value("YUV2BGR", YUV2BGR);
  eConversion.value("YUV2RGB", YUV2RGB);

  eConversion.value("BayerBG2GRAY", BayerBG2GRAY);
  eConversion.value("BayerGB2GRAY", BayerGB2GRAY);
  eConversion.value("BayerRG2GRAY", BayerRG2GRAY);
  eConversion.value("BayerGR2GRAY", BayerGR2GRAY);

  eConversion.export_values();

  bp::enum_<Interpolation> eInterpolation("Interpolation");
  eInterpolation.value("NN", NN);
  eInterpolation.value("LINEAR", LINEAR);
  eInterpolation.value("CUBIC", CUBIC);
  eInterpolation.value("AREA", AREA);
  eInterpolation.value("LANCZOS4", LANCZOS4);
  eInterpolation.export_values();

  bp::enum_<Morph> eMorph("Morph");
  eMorph.value("RECT", RECT);
  eMorph.value("CROSS", CROSS);
  eMorph.value("ELLIPSE", ELLIPSE);
  eMorph.export_values();
}
