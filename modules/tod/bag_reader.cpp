/*
 * bag_reader.cpp
 *
 *  Created on: Jun 23, 2011
 *      Author: vrabaud
 */

#include <iostream>

#include <pcl_ros/point_cloud.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>

#include <ecto/ecto.hpp>

#include "tod_stub/tod_stub.hpp"

struct BagReader
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<std::string>("path", "The path of the bag");
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    bag_path_ = params.get<std::string>("path");
    outputs.declare<cv::Mat>("img", "The image that will be analyzed");
    outputs.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>("pcd", "The point cloud");
    outputs.declare<cv::Mat>("K", "K for the camera");
    outputs.declare<cv::Mat>("D", "D for the camera");
  }

  void configure(ecto::tendrils& params)
  {
  }

  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    std::vector<std::string> topics;
    topics.push_back("image_mono");
    topics.push_back("camera_info");
    topics.push_back("points");

    for (size_t i = 0; i < topics.size(); i++)
      std::cout << "looking at topic:" << topics[i] << std::endl;

    rosbag::Bag bag;
    bag.open(bag_path_, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ImagePointsCamera ipc_package;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
          sensor_msgs::ImageConstPtr img = m.instantiate<sensor_msgs::Image>();
          if (img != NULL)
          {
            ipc_package.img = img;
          }
          sensor_msgs::CameraInfoConstPtr camera_info = m.instantiate<sensor_msgs::CameraInfo>();
          if (camera_info != NULL)
          {
            ipc_package.camera_info = camera_info;
          }

          Cloud_t::ConstPtr points2 = m.instantiate<Cloud_t>();
          if (points2 != NULL)
          {
            ipc_package.points2 = points2;
          }
          tod_training::PoseRTConstPtr pose = m.instantiate<tod_training::PoseRT>();
          if (pose != NULL)
          {
            ipc_package.pose = pose;
          }

          if (ipc_package.full() && (!has_pose || ipc_package.pose != NULL))
          {
            try
            {
              processor(ipc_package);
            }
            catch (std::exception& e)
            {
              std::cerr << e.what() << std::endl;
              //return;
            }
            catch (tod_exception e)
            {
              switch (e)
              {
                case tod_stub::QUIT:
                  std::cout << "quit" << std::endl;
                  return;
                  break;
                case tod_stub::ERROR:
                  //std::cerr << "error" << std::endl;
                  break;
              }
            }
            ipc_package.clear();
          }
        }

    current_frame++;
    if (current_frame >= int(docs.size()))
      return ecto::QUIT;

    return 0;
  }
  std::string bag_path_;
};
