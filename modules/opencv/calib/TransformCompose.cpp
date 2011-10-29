#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using ecto::tendrils;
namespace calib
{
  struct TransformCompose
  {
    typedef TransformCompose C;
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&C::R1,"R1", "3x3 Rotation matrix.");
      in.declare(&C::T1,"T1", "3x1 Translation vector.");
      in.declare(&C::R2,"R2", "3x3 Rotation matrix.");
      in.declare(&C::T2,"T2", "3x1 Translation vector.");
      out.declare(&C::R,"R", "3x3 Rotation matrix.");
      out.declare(&C::T,"T", "3x1 Translation vector.");    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      //reset outputs to enable multithreading.
      *R = cv::Mat(); *T = cv::Mat();
      cv::composeRT(*R1,*T1,*R2,*T2,*R,*T);
      return ecto::OK;
    }
    ecto::spore<cv::Mat> R1,T1,R2,T2,R,T;
  };
}
using namespace calib;
ECTO_CELL(calib, TransformCompose, "TransformCompose", "Compose to transforms.");
