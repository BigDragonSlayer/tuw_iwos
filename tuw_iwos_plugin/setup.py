#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tuw_iwos_plugin'],
    package_dir={'': 'src'},
    scripts=['scripts/tuw_iwos_plugin']
)

setup(**d)
