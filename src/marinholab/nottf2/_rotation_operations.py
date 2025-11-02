import numpy as np
from math import isclose
from geometry_msgs.msg import Quaternion
from marinholab.nottf2._quaternion_operations import quaternion_norm


def rotation_inverse(r: Quaternion) -> Quaternion:
    """
    Calculate the inverse of a rotation quaternion. It supposes that the input quaternion has unit norm
    and returns an exception otherwise.

    :see: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    """
    n = quaternion_norm(r)
    if not isclose(n, 1.0):
        raise ValueError(f"[nottf2] Input quaternion for rotation_inverse must have unit norm {n}!={1.0}.")

    # Simply the quaternion conjugate.
    return Quaternion(w = r.w, x = -r.x, y = -r.y, z = -r.z)