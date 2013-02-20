writing opencv cells
=====================

Since opencv uses ``cv::Mat`` which behaves like a smart pointer, it is
important that cells not munge the output data by working in place.

One solution to this issue, so that cells may play nicely in multi threaded
mode:

.. code-block:: c++
  
    int
    process(const tendrils& i, const tendrils& o)
    {
      cv::Mat out,in;
      i["in"] >> in;
      
      //do some cv operation, be sure that the output is a local cv::Mat
      //so that it doesn't reuse the memory held in the output
      //tendril.
      cv::resize(in,out, cv::Size(64,64));
      
      //copy our mat into the output, using the << operator.
      //this is a copy by reference, for cv::Mat...
      o["out"] << out;
      return ecto::OK;
    }

**Do not use the following method** unless your cell is never going to be run in
a multi threaded app.

.. code-block:: c++
  
    int
    process(const tendrils& i, const tendrils& o)
    {
      cv::Mat in;
      i["in"] >> in;
      cv::resize(in,o.get<cv::Mat>("out"), cv::Size(64,64));
      return ecto::OK;
    }

The reason for this is that the opencv will work in place on the the memory held
by the output tendril. This will happen without any locking mechanisms, and cells
that use the output downstream may be affected in undefined ways.