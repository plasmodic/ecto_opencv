#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>

using ecto::tendrils;
namespace imgproc
{
  struct Pairwise
  {

    Pairwise() : 
      seq(0)
    {}

    static void declare_params(tendrils& p) {}

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("image", "The n'th image");
      outputs.declare<cv::Mat>("image1", "The (n-1)'th image unless n=0, then n'th image. ");
      outputs.declare<cv::Mat>("image2", "The n'th image");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      input = inputs["image"];
      output1 = outputs["image1"];
      output2 = outputs["image2"];
    }

    int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      // On the first processing call
      if(seq == 0)
	{
	  // Allocate an image like th eoe passed.
	  temp_storage = new cv::Mat(*input);
	}

      // Setup the return (output) values
      *output1 = *input;
      *output2 = *temp_storage;

      // Don't need to copy on the first pass, already allocated from that
      if(seq != 0)
	{    
	  // Otherwise, copy the current imput over the previous
	  input->copyTo(*temp_storage);
	}
      
      // Update the sequence number:
      // TODO: present this number as a return?
      seq += 1;
      return ecto::OK;
    }

    size_t seq;
    cv::Mat *temp_storage;
    ecto::spore<cv::Mat> input, output1, output2;
  };
}

// Not a stateless cell, so make sure there is only one instance at a time of this . . .
ECTO_THREAD_UNSAFE(imgproc::Pairwise);

ECTO_CELL(imgproc, imgproc::Pairwise, "Pairwise", "Cell for doing pairwise image comparison of sequential images.\n First return duplicates the first input.  Later calls include the current image and the last that made its way throught the pipeline.");
