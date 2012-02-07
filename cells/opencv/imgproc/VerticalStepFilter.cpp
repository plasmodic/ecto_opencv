#include <ecto/ecto.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgproc.h"
#include <stdlib.h>

using ecto::tendrils;
using ecto::spore;
using namespace cv;

namespace imgproc
{
  unsigned short int pat[20] = {0, 1, 1, 1,
			  0, 1, 0, 1,
			  0, 0, 1, 1,
			  1, 0, 1, 1,
			  1, 1, 1, 1};

  // dithers either side of a column, out to a distance dsize
  // col is the index of column just before the step
  // step is the estimated size of the step
  void ditherColumn(Mat &src, int col, int step)
  {
    const int dsize = 4;
    int rows = src.rows;
    int cols = src.cols;
    if (col-dsize+1 < 0 || col+dsize >= cols) return; // too close to left or right side
    for (int i=0; i<rows; i++)
      {
	unsigned short* Mi = src.ptr<unsigned short>(i);
	int m = rand() % 5;	// [0,4]
	unsigned short int *pp = &pat[m*4];

	// add in values on left side
	int j=col-dsize+1;
	for (; j<=col; j++)
	  Mi[j] += *pp++;

	if (step == 1) continue;

	// subtract values on right side
	j += dsize;
	m = rand() % 5;	// [0,4]
	pp = &pat[m*4];
	for (; j>col; j--)
	  Mi[j] -= *pp++;
      }
  }

  void ditherRow(Mat &src, int col, int row)
  {
    const int dsize = 4;
    short* Mi = src.ptr<short>(row);
    int step = Mi[col+1] - Mi[col];

    if (step < 1 || step > 2) return;

    int m = rand() % 5;	// [0,4]
    unsigned short int *pp = &pat[m*4];

    // add in values on left side
    int j=col-dsize+1;
    for (; j<=col; j++)
      Mi[j] += *pp++;

    if (step == 1) return;

    // subtract values on right side
    j += dsize;
    m = rand() % 5;	// [0,4]
    pp = &pat[m*4];
    for (; j>col; j--)
      Mi[j] -= *pp++;
  }

  void ditherRowN(Mat &src, int col, int row)
  {
    const int dsize = 4;
    short* Mi = src.ptr<short>(row);
    int step = Mi[col] - Mi[col+1];

    if (step < 1 || step > 2) return;

    int m = rand() % 5;	// [0,4]
    unsigned short int *pp = &pat[m*4];

    // subtract values on left side
    int j=col-dsize+1;
    for (; j<=col; j++)
      Mi[j] -= *pp++;

    if (step == 1) return;

    // add values on right side
    j += dsize;
    m = rand() % 5;	// [0,4]
    pp = &pat[m*4];
    for (; j>col; j--)
      Mi[j] += *pp++;
  }

  // finds a vertical step using a FIR filter, looking for 1 and 2 step intervals between neighboring cols
  void findVerticalStep(Mat &src, int col, int fsize, int thresh)
  {
    char fbuf[fsize];
    char fbufn[fsize];
    for (int i=0; i<fsize; i++) {fbuf[i] = 0; fbufn[i] = 0;} // init
    int tot = 0, totn = 0;	// total number of responses
    int ind = 0;		// end of FIR
    int out = 0, outn = 0;

    int rows = src.rows;
    for (int i=0; i<rows; i++)
      {
	short* Mi = src.ptr<short>(i);
	int diff = Mi[col+1] - Mi[col];
	if (diff < 3 && diff > 0)
	  {
	    tot++;
	    fbuf[ind] = diff;
	  }
	if (diff > -3 && diff < 0)
	  {
	    totn++;
	    fbufn[ind] = diff;
	  }

	if (++ind >= fsize) ind = 0;
	if (fbuf[ind]!=0) tot--; // fall off the back
	if (fbufn[ind]!=0) totn--; // fall off the back
	fbuf[ind] = 0;		// reset
	fbufn[ind] = 0;		// reset
	//	std::cout << Mi[col] << " " << Mi[col+1] << " " << tot << std::endl;
	if (tot > thresh)
	  {
	    ditherRow(src, col, i-fsize+tot);
	    out++;
	  }
	if (totn > thresh)
	  {
	    ditherRowN(src, col, i-fsize+tot);
	    outn++;
	  }

      }    
    //    if (out > 0 || outn > 0) std::cout << col << " " << out << " " << outn << std::endl;
  }

  struct VerticalStepFilter
  {
    typedef VerticalStepFilter vf;

    static void
    declare_params(tendrils& p)
    {
      p.declare(&vf::fir_size_,"fir_size", "The size of the FIR, in pixels", 20);
      p.declare(&vf::fir_frac_,"fir_frac", "The fraction of the FIR for a valid response", 0.6);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      fsize = *fir_size_;
      thresh = *fir_size_ * *fir_frac_;
    }

    int
    process(const tendrils&, const tendrils&, const cv::Mat& input, cv::Mat& output)
    {
      // assumes input and output are 16-bit depth images
      // looks for vertical steps
      output = input.clone();
      //      ditherColumn(output,99,2);
      //      ditherColumn(output,303,1);
      for (int i=0; i<output.cols; i++)
	findVerticalStep(output,i,fsize,thresh);
      return ecto::OK;
    }

    spore<double> fir_frac_;
    spore<int> fir_size_;
    int fsize;
    int thresh;
  };
}

using namespace imgproc;
ECTO_CELL(imgproc, Filter_<VerticalStepFilter>, "VerticalStepFilter", "Applies a vertical step filter to depth images with this noise");
