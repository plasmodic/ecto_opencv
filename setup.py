#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['packages'] = ['ecto_opencv', 'ecto_opencv.features2d']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
