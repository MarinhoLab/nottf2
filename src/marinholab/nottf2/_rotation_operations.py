"""
Copyright (c) 2025. Murilo Marques Marinho (murilomarinho.info)
MIT License.
"""
import numpy as np
from math import isclose, sin, cos
from geometry_msgs.msg import Quaternion
from marinholab.nottf2._quaternion_operations import quaternion_norm


def rx(phi: float) -> Quaternion:
    """
    Return the quaternion representing a rotation of phi radians about the x-axis.
    :param phi: rotation angle, in radians.
    :return: the rotation quaternion.
    """
    return Quaternion(w = cos(phi/2.0),
                      x = sin(phi/2.0),
                      y = 0,
                      z = 0)

def ry(phi: float) -> Quaternion:
    """
    Return the quaternion representing a rotation of phi radians about the y-axis.
    :param phi: rotation angle, in radians.
    :return: the rotation quaternion.
    """
    return Quaternion(w = cos(phi/2.0),
                      x = 0,
                      y = sin(phi/2.0),
                      z = 0)

def rz(phi: float) -> Quaternion:
    """
    Return the quaternion representing a rotation of phi radians about the z-axis.
    :param phi: rotation angle, in radians.
    :return: the rotation quaternion.
    """
    return Quaternion(w = cos(phi/2.0),
                      x = 0,
                      y = 0,
                      z = sin(phi/2.0))

def rotation_inverse(r: Quaternion) -> Quaternion:
    """
    Calculate the inverse of a rotation quaternion. It supposes that the input quaternion has unit norm
    and returns an exception otherwise.

    :param r: rotation quaternion.
    :return: rotation quaternion inverse (its conjugate).
    :see: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    """
    n = quaternion_norm(r)
    if not isclose(n, 1.0):
        raise ValueError(f"[nottf2] Input quaternion for rotation_inverse must have unit norm {n}!={1.0}.")

    # Simply the quaternion conjugate.
    return Quaternion(w = r.w, x = -r.x, y = -r.y, z = -r.z)