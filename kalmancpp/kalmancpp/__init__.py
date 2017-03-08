"""
kalmancpp
============

Package containing the class definition for a linear kalman filter implemented in Cpp and wrapped in Cython


"""

# init file

# the following 8 lines set the __version__ variable
import pkg_resources as _pkg_resources
import subprocess as _subprocess
import os as _os

_resource_package = __name__  # Gets the module/package name.
_cmd = "make"
_PackageDirectory = _pkg_resources.resource_filename(_resource_package, '')
with open(_os.devnull, 'w') as _fp:
    _subprocess.call(_cmd, cwd=_PackageDirectory, stdout=_fp, stderr=_fp)

_resource_package = __name__  # Gets the module/package name.
_rawVersionFile = _pkg_resources.resource_string(
            _resource_package, "VERSION")
_decodedVersionFile = _rawVersionFile.decode(encoding='UTF-8')
# decocdes the bytes object into a string object
__version__ = _decodedVersionFile.splitlines()[0]

# the following line imports all the functions from DataHandling.py
from .KalmanFilter import *
