#
# Copyright 2008,2009 Free Software Foundation, Inc.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

# The presence of this file turns this directory into a Python package

'''
This is the GNU Radio OPENCV module. Place your Python package
description here (python/__init__.py).
'''
import os

# import pybind11 generated symbols into the opencv namespace
try:
    # this might fail if the module is python-only
    from .opencv_python import *
except ModuleNotFoundError:
    pass

# import any pure python here
from .video_source import video_source
from .video_sink import video_sink
from .aruco_detector import aruco_detector
#
