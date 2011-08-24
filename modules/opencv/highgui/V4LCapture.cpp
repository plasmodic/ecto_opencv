
#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>
using ecto::tendrils;
#include "capture_interface.hpp"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "libv4l2.h"

namespace ecto_opencv
{

  struct V4LCapture
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("video_device", "The device path to open.", std::string("/dev/video0"));
      params.declare<unsigned>("width", "Width", 640);
      params.declare<unsigned>("height", "Height", 480);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      //set outputs
      declare_video_device_outputs(outputs);
    }

#define CLEAR(x) memset(&(x), 0, sizeof(x))

    static void xioctl(int fh, int request, void *arg)
    {
      int r;

      do {
        r = v4l2_ioctl(fh, request, arg);
      } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

      if (r == -1) {
        fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE);
      }
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      video_device = params.get<std::string>("video_device");
      width = params.get<unsigned>("width");
      height = params.get<unsigned>("height");
      frame_number_ = outputs["frame_number"];

      fd = v4l2_open(video_device.c_str(), O_RDWR| O_NONBLOCK, 0);
      if (fd < 0) {
        perror("Cannot open device");
      }
      std::cout << "fd=" << fd << "\n";

      CLEAR(fmt);
      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width       = width;
      fmt.fmt.pix.height      = height;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

      xioctl(fd, VIDIOC_S_FMT, &fmt);
      if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
        printf("Libv4l didn't accept RGB24 format. Can't proceed.\n");
        exit(EXIT_FAILURE);
      }
      if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height))
        printf("Warning: driver is sending image at %dx%d\n",
               fmt.fmt.pix.width, fmt.fmt.pix.height);

      std::cout << "Driver sending video at " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << "\n";

      v4l2_requestbuffers req;
      CLEAR(req);
      req.count = 2;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_MMAP;
      xioctl(fd, VIDIOC_REQBUFS, &req);

      std::cout << "Allocing " << req.count << " buffers.\n";
      buffers = (buffer*) calloc(req.count, sizeof(*buffers));

      for (unsigned n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        xioctl(fd, VIDIOC_QUERYBUF, &buf);

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
                                             PROT_READ | PROT_WRITE, MAP_SHARED,
                                             fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start) {
          perror("mmap");
          exit(EXIT_FAILURE);
        }
      }
      std::cout << "Alloced.\n";
      for (unsigned i = 0; i < req.count; ++i) {
        std::cout << "i=" << i << "\n";
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        xioctl(fd, VIDIOC_QBUF, &buf);
      }
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      std::cout << "starting stream\n";
      xioctl(fd, VIDIOC_STREAMON, &type);
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat image;
      fd_set fds;
      timeval tv;
      int r;
      do {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);
      } while ((r == -1 && (errno = EINTR)));
      if (r == -1) {
        perror("select");
        return errno;
      }

      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      xioctl(fd, VIDIOC_DQBUF, &buf);

      xioctl(fd, VIDIOC_QBUF, &buf);

      cv::Mat tmp(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC3);
      memcpy(tmp.data, buffers[buf.index].start, buf.bytesused);


      outputs["image"] << tmp;

      //increment our frame number.
      ++(*frame_number_);
      return 0;
    }

    ecto::spore<int> frame_number_;
    struct buffer {
      void   *start;
      size_t length;
    };

    buffer *buffers;
    v4l2_buffer buf;
    std::string video_device;
    unsigned width, height;
    int fd;
    struct v4l2_format              fmt;

  };
}
ECTO_CELL(highgui, ecto_opencv::V4LCapture, "V4LCapture", "Read images from a directory.");
