/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <stdio.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "interfaces.h"

std::string
temporary_yml_file_name()
{
  std::string fname;
  {
    char buffer[L_tmpnam];
    char* p = std::tmpnam(buffer);
    if (p != NULL)
    {
      fname = std::string(buffer) + ".yml";
    }
    else
      throw std::runtime_error("Could not create temporary filename!");
  }
  return fname;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
read_tendrils_as_file_node(const ecto::tendrils & tendrils, boost::function<void
(const cv::FileNode &)> function)
{
  // Get the file
  std::string file_name = temporary_yml_file_name();

  // Write the tendril's to it
  {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    BOOST_FOREACH (const ecto::tendrils::value_type &tendril_pair, tendrils)
        {
          std::string tendril_name = tendril_pair.first;
          const ecto::tendril & tendril = *(tendril_pair.second);
          std::string type_name = tendril.type_name();
          fs << tendril_name;
          if (type_name == "int")
            fs << tendril.get<int>();
          else if (type_name == "float")
            fs << tendril.get<float>();
          else
          {
            std::string error_message = "Unsupported type: ";
            error_message += type_name;
            throw std::runtime_error(error_message);
          }
        }
  }

  {
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    function(fs.root());
  }

  // Remove the temporary file
  boost::filesystem::remove(file_name.c_str());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
SIFT_interface::declare_common_params(tendrils&p)
{
#if (CV_MAJOR_VERSION > 2) || ((CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4))
  p.declare<int>("nOctaveLayers", "", 3);
#else
  int a = cv::SIFT::CommonParams::DEFAULT_NOCTAVES, b = cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, c =
      cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, d = cv::SIFT::CommonParams::FIRST_ANGLE;
  p.declare<int>("nOctaves", "", a);
  p.declare<int>("nOctaveLayers", "", b);
  p.declare<int>("firstOctave", "", c);
  p.declare<int>("angleMode", "", d);
#endif
}

