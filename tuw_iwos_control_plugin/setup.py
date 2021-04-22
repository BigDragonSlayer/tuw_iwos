#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tuw_iwos_control_plugin'],
    package_dir={'': 'src'},
    scripts=['scripts/tuw_iwos_control_plugin']
)

setup(**d)
