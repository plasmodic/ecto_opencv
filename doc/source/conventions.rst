ecto_opencv conventions
=======================

tendrils for opencv based cells should follow the following rules.

Naming
^^^^^^
In general, the name of opencv tendrils is determined by what they are as datatypes and
not what they do.

* ``image`` for any image, unless it must be further disambiguated, e.g. in the
  case of image, and depth, and may be of common image type.
* ``mask`` for an image based binary mask, and be of type CV_8UC1. Generally this should be an optional input.
* ``points`` for any collection of 2d or 3d points, and the should be an NxM multichannel matrix where the number of dimensions is apparent in number of channels.
* ``R`` is a 3 by 3 rotation matrix
* ``T`` is a 3 by 1 translation vector
* ``K`` is a 3 by 3 camera intrinsic matrix
