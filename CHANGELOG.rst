0.7.0 (2016-04-24)
------------------
* make master Kinetic and above only
  This is because OpenCV3 is the default and cv_backports is now
  useless.
* Contributors: Vincent Rabaud

0.6.2 (2016-04-24)
------------------
* fix compilation with OpenCV3
* Fix version check logic
* Contributors: Scott K Logan, Vincent Rabaud

0.6.1 (2015-08-31)
------------------
* Fix compatibility with OpenCV 2.4.11+
* Contributors: Scott K Logan, Vincent Rabaud

0.6.0 (2015-05-09)
------------------
* add proper rosunit dependency
* convert Python tests to proper nose tests
* use cv_backports to pull in opencv2.4.9 imshows with zoom and parallel thread support.
* Contributors: Daniel Stonier, Vincent Rabaud

0.5.6 (2015-04-19)
------------------
* fix non-existence of radiusMatch for LSH
* Contributors: Vincent Rabaud

0.5.5 (2015-03-29)
------------------
* compile with the latest opencv_candidate
* use OpenCV's LSH
* remove useless file
* clean extensions
* Contributors: Vincent Rabaud

0.5.4 (2015-01-03)
------------------
* remove SSE dependency
  That would not work on ARM as explained in `#29 <https://github.com/plasmodic/ecto_opencv/issues/29>`_ plus it is useless
  now: opencv_candidate deals with it and it uses OpenCV flags
* Contributors: Vincent Rabaud

0.5.3 (2014-07-27)
------------------
* Added FPS scale param
* recursive directory creation.
* speedup normal display
* fix depth cleaner
* add a normal sample
* Contributors: Daniel Stonier, Vincent Rabaud, kentsommer

0.5.2 (2014-04-13)
------------------
* rely on opencv_candidate to get the OpenCV dependency
* Contributors: Vincent Rabaud

0.5.1 (2014-04-13)
------------------
* get code to compile on Indigo
* fix shadow effect on clusters
* Contributors: Vincent Rabaud

0.5.0 (2013-12-18  21:12:06 +0100)
----------------------------------
- drop Fuerte support
- fix compatibility with Boost until 1.54
