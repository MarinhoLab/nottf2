from marinholab.nottf2._quaternion_operations import quaternion_norm, quaternion_multiply
from marinholab.nottf2._rotation_operations import rotation_inverse, rotx, roty, rotz

# https://setuptools-git-versioning.readthedocs.io/en/stable/runtime_version.html
from importlib.metadata import version, PackageNotFoundError
try:
    __version__ = version("nottf2")
except PackageNotFoundError:
    # package is not installed
    pass