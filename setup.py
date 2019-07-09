#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    ##  don't do this unless you want a globally visible script
    packages=['panda_eup','panda_gui'],
    package_dir={'': 'src'}
)

setup(**setup_args)