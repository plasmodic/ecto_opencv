#include <ecto/ecto.hpp>

#undef  BOOST_PARAMETER_MAX_ARITY
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <iostream>

using ecto::tendrils;

typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_t;
class SimpleKinectGrabber
{
public:
  SimpleKinectGrabber() :
    thread_(boost::ref(*this))
  {
  }

  ~SimpleKinectGrabber()
  {
  }

  void operator ()()
  {

    boost::scoped_ptr<pcl::Grabber> interface(new pcl::OpenNIGrabber());

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> point_cloud_cb =
        boost::bind(&SimpleKinectGrabber::cloud_cb_, this, _1);

    boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&,
                         const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> image_depth_cb =
        boost::bind(&SimpleKinectGrabber::image_depth_cb_, this, _1, _2, _3);

    //interface->
    boost::signals2::connection c = interface->registerCallback(point_cloud_cb);

    interface->start();

    while (!thread_.interruption_requested())
      {
        boost::thread::yield();
      }

    c.disconnect();
    std::cerr << "Stopping" << std::endl;

    interface->stop();
  }

  /**
   * \brief don't hang on to this cloud!! or it won't get updated.
   */
  cloud_t::ConstPtr getLatestCloud()
  {
    boost::mutex::scoped_lock lock(mutex_);
    cloud_t::ConstPtr p = cloud_;
    cloud_.reset();
    return p;
  }
  //depth callback.
  void image_depth_cb_(const boost::shared_ptr<openni_wrapper::Image>&,
                       const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)
  {

  }
  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cloud_ = cloud;
  }

  boost::mutex mutex_;
  cloud_t::ConstPtr cloud_;
  boost::thread thread_;

};

struct VoxelGrid
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<float> ("leaf_size", "The size of the leaf(meters), smaller means more points...", 0.05);

  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to filter");
    outputs.declare<cloud_t::ConstPtr> ("output", "Filtered cloud.");

  }
  VoxelGrid() :
    cloud_out_(new cloud_t)
  {
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    //set the voxel grid size
    float leaf_size = params.get<float> ("leaf_size");
    voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    //grab the input cloud
    cloud_t::ConstPtr cloud = inputs.get<cloud_t::ConstPtr> ("input");
    //voxel grid filter it.
    voxel_grid_.setInputCloud(cloud);
    voxel_grid_.filter(*cloud_out_);
    //set the output to the voxelized cloud.
    outputs.get<cloud_t::ConstPtr> ("output") = cloud_out_;
    return 0;
  }
  pcl::VoxelGrid<cloud_t::PointType> voxel_grid_;
  cloud_t::Ptr cloud_out_;

};

struct KinectGrabber
{
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    outputs.declare<cloud_t::ConstPtr> ("output", "An rgb xyz point cloud from the kinect");
  }

  int process(const tendrils& /*inputs*/, tendrils& outputs)
  {
    cloud_t::ConstPtr p;
    while (!p)
      {
        p = grabber_.getLatestCloud();
        boost::thread::yield();
      }
    outputs.get<cloud_t::ConstPtr> ("output") = p;
    return 0;
  }

  SimpleKinectGrabber grabber_;
};

struct CloudViewer
{
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<std::string> ("window_name", "The window name", "cloud viewer");

  }
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cloud_t::ConstPtr> ("input", "The cloud to view");
    outputs.declare<bool> ("stop", "True if stop requested", false);
  }
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    viewer_.reset(new pcl::visualization::CloudViewer(params.get<std::string> ("window_name")));
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    if (!viewer_)
      return 1;
    cloud_t::ConstPtr cloud = inputs.get<cloud_t::ConstPtr> ("input");
    if (cloud)
      viewer_->showCloud(cloud, "cloud");
    if (viewer_->wasStopped(10))
      outputs.get<bool> ("stop") = true;
    return 0;
  }
  boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;
};

BOOST_PYTHON_MODULE(pcl)
{
  ecto::wrap<VoxelGrid>("VoxelGrid", "Does a voxel grid downsampling of a point cloud.");
  ecto::wrap<KinectGrabber>("KinectGrabber", "This grabs frames from the kinect!!!");
  ecto::wrap<CloudViewer>("CloudViewer", "View a point cloud.");
}

