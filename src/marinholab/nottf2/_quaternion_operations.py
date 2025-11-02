"""
Copyright (c) 2025. Murilo Marques Marinho (murilomarinho.info)
MIT License.
"""
import numpy as np

try:
    from geometry_msgs.msg import Quaternion
except ModuleNotFoundError as e:
    print("This module needs `ros2` to be installed. Please install `ros2` and properly source the environment.")
    raise e

def _quaternion_to_ndarray(a: Quaternion) -> np.ndarray:
    return np.ndarray([a.w, a.x, a.y, a.z])

def _ndarray_to_quaternion(a_vec: np.ndarray) -> Quaternion:
    return Quaternion(w = a_vec[0],
                      x = a_vec[1],
                      y = a_vec[2],
                      z = a_vec[3])

def quaternion_norm(a: Quaternion) -> float:
    """
    Calculate the norm of the quaternion.

    This function relies on numpy's efficient implementation of the norm to obtain the quaternion norm.
    :param a: Quaternion
    :return: norm of `a`.
    """
    a_vector = _quaternion_to_ndarray(a)
    return np.linalg.norm(a_vector)

def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """
    Calculate the quaternion multiplication c = ab.

    This function relies on numpy's efficient implementation of the multiplication operation between a matrix and
    a vector.

    :param a: The first Quaternion.
    :param b: The second Quaternion.
    :return: Result of the quaternion multiplication of `a` and `b`. Please note that it is not commutative in general.
    :see: https://en.wikipedia.org/wiki/Quaternion
    """

    # Assigning to local variables for readability.
    aw = a.w
    ax = a.x
    ay = a.y
    az = a.z

    # In Python, it will be more efficient to rely on numpy to multiply matrices for us as it's highly optimized.
    # We will take advantage of that here.

    ## See https://en.wikipedia.org/wiki/Quaternion#Representation_as_complex_2_×_2_matrices
    H_plus_a = np.ndarray([[aw, -ax, -ay, -az],
                           [ax, aw, -az, ay],
                           [ay, az, aw, -ax],
                           [az, -ay, ax, aw]])

    ## We obtain the vector representation of the second quaternion.
    b_vector = _quaternion_to_ndarray(b)

    ## Multiply using numpy
    c_vector = H_plus_a @ b_vector

    # Return the result as a Quaternion.
    return _ndarray_to_quaternion(c_vector)




