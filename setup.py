#!/usr/bin/env python

from distutils.core import setup
from ecto_catkin.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['ecto_opencv', 'ecto_opencv.features2d']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
