#!/usr/bin/env python
import ecto
# just to test that the cv_bp import works
from ecto_opencv import cv_bp
from ecto_opencv import calib, features2d, imgproc, highgui
import unittest

class TestListModules(unittest.TestCase):

    def test_list_modules(self):
        ecto.list_ecto_module(features2d)
        ecto.list_ecto_module(highgui)
        ecto.list_ecto_module(calib)
        ecto.list_ecto_module(imgproc)

if __name__ == '__main__':
    import rosunit
    rosunit.rosrun('ecto_opencv', 'test_ecto_opencv', TestListModules)
