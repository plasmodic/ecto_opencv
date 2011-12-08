#pragma once
#include <opencv2/highgui/highgui.hpp>
namespace ecto_opencv
{
  namespace Record
  {
    enum RecordCommands
    {
      START = 0, RESUME = 1, PAUSE = 2, STOP = 3,
    };

  }
  namespace Image
  {
    enum Modes
    {
      /* 8bit, color or not */
      UNCHANGED = CV_LOAD_IMAGE_UNCHANGED,
      /* 8bit, gray */
      GRAYSCALE = CV_LOAD_IMAGE_GRAYSCALE,
      /* ?, color */
      COLOR = CV_LOAD_IMAGE_COLOR,
      /* any depth, ? */
      ANYDEPTH = CV_LOAD_IMAGE_ANYDEPTH,
      /* ?, any color */
      ANYCOLOR = CV_LOAD_IMAGE_ANYCOLOR
    };
  }
}
