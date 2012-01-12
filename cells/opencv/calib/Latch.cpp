#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
using ecto::tendrils;
namespace calib
{
  template<typename T>
  struct Latch
  {
    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare(&Latch::in_, "input", "The input to copy to the output..").required(true);
      in.declare(&Latch::set_, "set", "The latch a value.", false);
      in.declare(&Latch::reset_, "reset", "The latch a value.", false);
      out.declare(&Latch::out_, "output", "A copy of the input.");
      out.declare(&Latch::set_out_, "set", "Is the output set.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      if (*reset_)
      {
        *reset_ = false;
        *set_ = false;
        *set_out_ = false;
        *out_ = T();
      }
      if (*set_)
      {
        *out_ = *in_;
        *set_out_ = true;
      }
      return ecto::OK;
    }
    ecto::spore<T> in_, out_;
    ecto::spore<bool> set_, reset_,set_out_;
  };
}

ECTO_CELL(calib, calib::Latch<bool>, "LatchBool", "Latch a bool.");
ECTO_CELL(calib, calib::Latch<cv::Mat>, "LatchMat", "Latch a cv::Mat.");
