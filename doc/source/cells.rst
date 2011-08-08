.. _ecto_opencv:

ecto_opencv Cells
=================

These are all of the cells available in ecto_opencv. The different modules roughly
mirror the modules in opencv.

.. _ecto_opencv.calib:

ecto_opencv.calib
-----------------
The calib module contains cells that deal with camera calibration, fiducial pose
estimation, and structure from motion.


.. ectocell:: ecto_opencv.calib CameraCalibrator


.. ectocell:: ecto_opencv.calib CameraIntrinsics


.. ectocell:: ecto_opencv.calib CircleDrawer


.. ectocell:: ecto_opencv.calib DepthTo3d


.. ectocell:: ecto_opencv.calib FiducialPoseFinder


.. ectocell:: ecto_opencv.calib GatherPoints


.. ectocell:: ecto_opencv.calib PatternDetector


.. ectocell:: ecto_opencv.calib PatternDrawer


.. ectocell:: ecto_opencv.calib PingPongDetector


.. ectocell:: ecto_opencv.calib PlanarSegmentation


.. ectocell:: ecto_opencv.calib PoseDrawer


.. ectocell:: ecto_opencv.calib SubrectRectifier

.. _ecto_opencv.features2d:

ecto_opencv.features2d
----------------------
Feature detection cells.


.. ectocell:: ecto_opencv.features2d DrawKeypoints


.. ectocell:: ecto_opencv.features2d DrawMatches


.. ectocell:: ecto_opencv.features2d FAST


.. ectocell:: ecto_opencv.features2d FeatureDescriptor


.. ectocell:: ecto_opencv.features2d MatchRefinement


.. ectocell:: ecto_opencv.features2d Matcher


.. ectocell:: ecto_opencv.features2d ORB

.. _ecto_opencv.highgui:

ecto_opencv.highgui
-------------------
Common opencv image input and out, plus display.

.. ectocell:: ecto_opencv.highgui FPSDrawer


.. ectocell:: ecto_opencv.highgui ImageReader


.. ectocell:: ecto_opencv.highgui ImageSaver


.. ectocell:: ecto_opencv.highgui OpenNICapture


.. ectocell:: ecto_opencv.highgui VideoCapture


.. ectocell:: ecto_opencv.highgui imread


.. ectocell:: ecto_opencv.highgui imshow

.. _ecto_opencv.imgproc:

ecto_opencv.imgproc
-------------------
image manipulation.


.. ectocell:: ecto_opencv.imgproc AbsNormalized


.. ectocell:: ecto_opencv.imgproc Adder


.. ectocell:: ecto_opencv.imgproc BitwiseAnd


.. ectocell:: ecto_opencv.imgproc BitwiseNot


.. ectocell:: ecto_opencv.imgproc CartToPolar


.. ectocell:: ecto_opencv.imgproc ChannelSplitter


.. ectocell:: ecto_opencv.imgproc GaussianBlur


.. ectocell:: ecto_opencv.imgproc KMeansGradient


.. ectocell:: ecto_opencv.imgproc Scale


.. ectocell:: ecto_opencv.imgproc Scharr


.. ectocell:: ecto_opencv.imgproc Sobel


.. ectocell:: ecto_opencv.imgproc Translate


.. ectocell:: ecto_opencv.imgproc cvtColor

.. _ecto_opencv.line_mod:

ecto_opencv.line_mod
--------------------


.. ectocell:: ecto_opencv.line_mod ColorDebug


.. ectocell:: ecto_opencv.line_mod ColorMod


.. ectocell:: ecto_opencv.line_mod ColorTemplCalc


.. ectocell:: ecto_opencv.line_mod TestColorTempl


.. ectocell:: ecto_opencv.line_mod TrainColorTempl

.. _ecto_opencv.projector:

ecto_opencv.projector
---------------------


.. ectocell:: ecto_opencv.projector Calibrator


.. ectocell:: ecto_opencv.projector DepthWarper


.. ectocell:: ecto_opencv.projector FiducialWarper


.. ectocell:: ecto_opencv.projector ImageWarper


.. ectocell:: ecto_opencv.projector PatternProjector


.. ectocell:: ecto_opencv.projector PlaneFitter

