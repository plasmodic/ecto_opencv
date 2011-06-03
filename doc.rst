AbsNormalized (ecto::module)
=================================

A module...

inputs
---------------------------------

 - input [cv::Mat] default = None

    image.

outputs
---------------------------------

 - out [cv::Mat] default = None

    absolute and normalized


CameraCalibrator (ecto::module)
=================================

Accumulates observed points and ideal 3d points, and runs opencv calibration routines after some number of satisfactorily unique observations.

params
---------------------------------

 - n_obs [int] default = 50

    Number of observations

 - output_file_name [std::string] default = camera.yml

    The name of the camera calibration file

inputs
---------------------------------

 - found [bool] default = False

    TODO: doc str me.

 - ideal [std::vector<cv::Point3_<float>, std::allocator<cv::Point3_<float> > >] default = None

    The ideal object points.

 - image [cv::Mat] default = None

    The image that is being used for calibration

 - points [std::vector<cv::Point_<float>, std::allocator<cv::Point_<float> > >] default = None

    Image points

outputs
---------------------------------

 - calibrated [bool] default = False

    Done calibration

 - norm [float] default = 0.0

    Norm of the input points to all previous points observed.


CameraIntrinsics (ecto::module)
=================================

This reads a camera calibration file and puts the results on the outputs.

params
---------------------------------

 - camera_file [std::string] default = camera.yml

    The camera calibration file. Typically a .yml

outputs
---------------------------------

 - K [cv::Mat] default = None

    3x3 camera intrinsic matrix.

 - camera_model [std::string] default = pinhole

    The camera model. e.g pinhole,...

 - image_size [cv::Size_<int>] default = None

    The image size.


CartToPolar (ecto::module)
=================================

Takes x and y derivatives and does a polar coordinate tranform.

inputs
---------------------------------

 - x [cv::Mat] default = None

    x derivative image.

 - y [cv::Mat] default = None

    y derivative image.

outputs
---------------------------------

 - angle [cv::Mat] default = None

    The angle image.

 - magnitude [cv::Mat] default = None

    The magnitude image.


ChannelSplitter (ecto::module)
=================================

A module...

inputs
---------------------------------

 - input [cv::Mat] default = None

    The 3 channel image to split.

outputs
---------------------------------

 - out_0 [cv::Mat] default = None

    Channel 0.

 - out_1 [cv::Mat] default = None

    Channel 1.

 - out_2 [cv::Mat] default = None

    Channel 2.


CloudViewer (ecto::module)
=================================

View a point cloud.

params
---------------------------------

 - window_name [std::string] default = cloud viewer

    The window name

inputs
---------------------------------

 - input [boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const>] default = None

    The cloud to view

outputs
---------------------------------

 - stop [bool] default = False

    True if stop requested


DrawKeypoints (ecto::module)
=================================

A module...

inputs
---------------------------------

 - input [cv::Mat] default = None

    The input image, to draw over.

 - kpts [std::vector<cv::KeyPoint, std::allocator<cv::KeyPoint> >] default = None

    The keypoints to draw.

outputs
---------------------------------

 - output [cv::Mat] default = None

    The output image.


FAST (ecto::module)
=================================

Computes fast keypoints given an image, and mask.

params
---------------------------------

 - thresh [int] default = 20

    The FAST threshhold. 20 is a decent value.

inputs
---------------------------------

 - image [cv::Mat] default = None

    An input image.

 - mask [cv::Mat] default = None

    An mask, same size as image.

outputs
---------------------------------

 - kpts [std::vector<cv::KeyPoint, std::allocator<cv::KeyPoint> >] default = None

    The keypoints.


FiducialPoseFinder (ecto::module)
=================================

A module...

inputs
---------------------------------

 - K [cv::Mat] default = None

    The camera projection matrix.

 - found [bool] default = False

    TODO: doc str me.

 - ideal [std::vector<cv::Point3_<float>, std::allocator<cv::Point3_<float> > >] default = None

    The ideal object points.

 - points [std::vector<cv::Point_<float>, std::allocator<cv::Point_<float> > >] default = None

    Image points

outputs
---------------------------------

 - R [cv::Mat] default = None

    3x3 Rotation matrix.

 - T [cv::Mat] default = None

    3x1 Translation vector.


GaussianBlur (ecto::module)
=================================

Given an image, blurs it.

params
---------------------------------

 - kernel [int] default = 0

    kernel size, if zero computed from sigma

 - sigma [double] default = 1.0

    The first sigma in the guassian.

inputs
---------------------------------

 - input [cv::Mat] default = None

    image.

outputs
---------------------------------

 - out [cv::Mat] default = None

    blurred image


ImageAdder (ecto::module)
=================================

A module...

inputs
---------------------------------

 - a [cv::Mat] default = None

    to add to b

 - b [cv::Mat] default = None

    to add to a

outputs
---------------------------------

 - out [cv::Mat] default = None

    a + b


ImageReader (ecto::module)
=================================

Read images from a directory.

params
---------------------------------

 - ext [std::string] default = .png|.jpg|.bmp

    The image extension to look for.

 - path [std::string] default = /tmp/ecto/rules

    The path to read images from.

outputs
---------------------------------

 - frame_number [int] default = 0

    The number of frames captured.

 - out [cv::Mat] default = None

    A video frame.


KinectGrabber (ecto::module)
=================================

This grabs frames from the kinect!!!

outputs
---------------------------------

 - output [boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const>] default = None

    An rgb xyz point cloud from the kinect


ORB (ecto::module)
=================================

An ORB detector. Takes a image and a mask, and computes keypoints and descriptors(32 byte binary).

params
---------------------------------

 - n_features [int] default = 1000

    The number of desired features

 - n_levels [int] default = 3

    The number of scales

 - scale_factor [float] default = 1.20000004768

    The factor between scales

inputs
---------------------------------

 - image [cv::Mat] default = None

    An input image.

 - mask [cv::Mat] default = None

    An mask, same size as image.

outputs
---------------------------------

 - descriptors [cv::Mat] default = None

    The descriptors per keypoints

 - kpts [std::vector<cv::KeyPoint, std::allocator<cv::KeyPoint> >] default = None

    The keypoints.


OpenNICapture (ecto::module)
=================================

OpenNI capture device.

params
---------------------------------

 - video_mode [int] default = 0

    Video size mode

outputs
---------------------------------

 - depth [cv::Mat] default = None

    The output depth map

 - frame_number [int] default = 0

    The number of frames captured.

 - image [cv::Mat] default = None

    A video frame.


PatternDetector (ecto::module)
=================================

A module...

params
---------------------------------

 - cols [int] default = 11

    Number of dots in col direction

 - pattern_type [std::string] default = acircles

    The pattern type, possible values are: [chessboard|circles|acircles]

 - rows [int] default = 4

    Number of dots in row direction

 - square_size [float] default = 1.0

    The dimensions of each square

inputs
---------------------------------

 - input [cv::Mat] default = None

    The grayscale image to search for a calibration pattern in.

outputs
---------------------------------

 - found [bool] default = False

    Whether or not a pattern was found...

 - ideal [std::vector<cv::Point3_<float>, std::allocator<cv::Point3_<float> > >] default = None

    The ideal pattern points.

 - out [std::vector<cv::Point_<float>, std::allocator<cv::Point_<float> > >] default = None

    The observed pattern points.


PatternDrawer (ecto::module)
=================================

A module...

params
---------------------------------

 - cols [int] default = 11

    Number of dots in col direction

 - rows [int] default = 4

    Number of dots in row direction

inputs
---------------------------------

 - found [bool] default = False

    Found the pattern

 - input [cv::Mat] default = None

    The image to to find a vertical lazer line in.

 - points [std::vector<cv::Point_<float>, std::allocator<cv::Point_<float> > >] default = None

    Circle pattern points.

outputs
---------------------------------

 - out [cv::Mat] default = None

    Pattern Image


PlanarSegmentation (ecto::module)
=================================

Given a pose, assuming it describes the center of the object coordinate system and lies on a plane, segment the object from the plane

params
---------------------------------

 - x_crop [float] default = 0.25

    The amount to keep in the x direction (meters) relative to the coordinate frame defined by the pose.

 - y_crop [float] default = 0.25

    The amount to keep in the y direction (meters) relative to the coordinate frame defined by the pose.

 - z_crop [float] default = 0.25

    The amount to keep in the z direction (meters) relative to the coordinate frame defined by the pose.

inputs
---------------------------------

 - R [cv::Mat] default = None

    The pose rotation matrix

 - T [cv::Mat] default = None

    The pose traslation vector

 - depth [cv::Mat] default = None

    The depth image to segment

outputs
---------------------------------

 - mask [cv::Mat] default = None

    The output mask, determined by the segmentation. 255 is the value for objects satisfying the constraints. 0 otherwise.


PoseDrawer (ecto::module)
=================================

A module...

inputs
---------------------------------

 - K [cv::Mat] default = None

    The camera projection matrix.

 - R [cv::Mat] default = None

    3x3 Rotation matrix.

 - T [cv::Mat] default = None

    3x1 Translation vector.

 - image [cv::Mat] default = None

    The original image to draw the pose onto.

outputs
---------------------------------

 - output [cv::Mat] default = None

    The pose of the fiducial, drawn on an image


ScanLineDrawer (ecto::module)
=================================

Draws a scanline in the image.
Uses the intensity on the y axis, x position on the x axis.

params
---------------------------------

 - auto_scan [bool] default = True

    After each process, increment the scanline

 - scan_idx [float] default = 0.5

    The scan line index, [0,1]

inputs
---------------------------------

 - in [cv::Mat] default = None

    The image to draw a scan line from.

outputs
---------------------------------

 - out [cv::Mat] default = None

    The scan line image.


Sobel (ecto::module)
=================================

A module...

params
---------------------------------

 - x [int] default = 0

    The derivative order in the x direction

 - y [int] default = 0

    The derivative order in the y direction

inputs
---------------------------------

 - input [cv::Mat] default = None

    image.

outputs
---------------------------------

 - out [cv::Mat] default = None

    sobel image


VideoCapture (ecto::module)
=================================

Use to capture video from a camera or video file.

params
---------------------------------

 - video_device [int] default = 0

    The device ID to open.

 - video_file [std::string] default = 

    A video file to read, leave empty to open a video device.

outputs
---------------------------------

 - frame_number [int] default = 0

    The number of frames captured.

 - image [cv::Mat] default = None

    A video frame.


VoxelGrid (ecto::module)
=================================

Does a voxel grid downsampling of a point cloud.

params
---------------------------------

 - leaf_size [float] default = 0.0500000007451

    The size of the leaf(meters), smaller means more points...

inputs
---------------------------------

 - input [boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const>] default = None

    The cloud to filter

outputs
---------------------------------

 - output [boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> const>] default = None

    Filtered cloud.


cvtColor (ecto::module)
=================================

A module...

params
---------------------------------

 - flag [int] default = 4

    Convert an image's color using opencv, possible flags are:
     RGB2GRAY = 7
     RGB2BGR = 4
     RGB2LAB = 45
     BGR2LAB = 44

inputs
---------------------------------

 - input [cv::Mat] default = None

    Color image.

outputs
---------------------------------

 - out [cv::Mat] default = None

    input as a Gray image.


imshow (ecto::module)
=================================

Shows an image in a named window.

params
---------------------------------

 - autoSize [bool] default = True

    Autosize the window.

 - name [std::string] default = image

    The window name

 - waitKey [int] default = -1

    Number of millis to wait, -1 for not at all, 0 for infinity.

inputs
---------------------------------

 - input [cv::Mat] default = None

    The image to show

outputs
---------------------------------

 - out [int] default = 0

    Character pressed.


