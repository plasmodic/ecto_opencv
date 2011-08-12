#pragma once
namespace ecto_opencv
{
  inline void
  declare_video_device_outputs(tendrils& outputs)
  {
    //set outputs
    outputs.declare < cv::Mat > ("image", "A video frame.", cv::Mat());
    outputs.declare<int>("frame_number", "The number of frames captured.", 0);
  }
}
