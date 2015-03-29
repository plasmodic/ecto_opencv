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
