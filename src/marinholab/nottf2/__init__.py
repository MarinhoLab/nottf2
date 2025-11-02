# https://setuptools-git-versioning.readthedocs.io/en/stable/runtime_version.html
from importlib.metadata import version, PackageNotFoundError
try:
    __version__ = version("nottf2")
except PackageNotFoundError:
    # package is not installed
    pass