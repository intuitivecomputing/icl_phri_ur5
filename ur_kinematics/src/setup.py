#!/usr/bin/env python

from distutils.core import setup
from distutils.extension import Extension

setup(name="ur_kinematics",
    ext_modules=[
        Extension("ur_kin_py", ["ur_kin_py.cpp"],
        libraries = ["boost_python"])
    ])
