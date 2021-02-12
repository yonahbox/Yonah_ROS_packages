#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['dynamic_delay', 'feedback_util', 'g2a', 'headers', 'regular', 'waypoint'],
     package_dir={'': 'scripts'}
)

setup(**setup_args)
