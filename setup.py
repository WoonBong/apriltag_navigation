#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Setup configuration
d = generate_distutils_setup(
    packages=['apriltag_navigation',
              'apriltag_navigation.map',
              'apriltag_navigation.perception',
              'apriltag_navigation.navigation',
              'apriltag_navigation.hardware'],
    package_dir={'': 'src'}
)

setup(**d)
