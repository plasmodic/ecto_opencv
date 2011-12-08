/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ecto/ecto.hpp>


#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/format.hpp>

#include "highgui.h"

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    void
    save(Archive & ar, const cv::Mat & m, const unsigned int version)
    {
      int type = m.type();
      ar & m.rows;
      ar & m.cols;
      ar & type;
      const uchar * data = m.data, *end = m.dataend;
      ar & boost::serialization::make_binary_object(const_cast<uchar*>(data), size_t(end - data));
    }

    template<class Archive>
    void
    load(Archive & ar, cv::Mat & m, const unsigned int version)
    {
      int rows, cols, type;
      ar & rows;
      ar & cols;
      ar & type;
      if (rows > 0 && cols > 0)
      {
        m.create(rows, cols, type);
        uchar * data = m.data, *end = m.dataend;
        ar & boost::serialization::make_binary_object(data, end - data);
      }
      else
      {
        std::cout << "bad matrix" << std::endl;
      }
    }
  } // namespace serialization
} // namespace boost

ECTO_DEFINE_MODULE(highgui)
{
boost::python::enum_<ecto_opencv::Record::RecordCommands>("RecordCommands")
    .value("START", ecto_opencv::Record::START)
    .value("RESUME", ecto_opencv::Record::RESUME)
    .value("PAUSE", ecto_opencv::Record::PAUSE)
    .value("STOP", ecto_opencv::Record::STOP)
    .export_values()
    ;
boost::python::enum_<ecto_opencv::Image::Modes>("ImageMode")
    .value("GRAYSCALE",ecto_opencv::Image::GRAYSCALE)
    .value("COLOR",ecto_opencv::Image::COLOR)
    .value("UNCHANGED",ecto_opencv::Image::UNCHANGED)
    .value("ANYCOLOR",ecto_opencv::Image::ANYCOLOR)
    .value("ANYDEPTH",ecto_opencv::Image::ANYDEPTH)
    .export_values()
    ;
}
