import numpy as np
import scipy


def isometry_inv(T):
    """
    Returns the inverse of a 4x4 matrix representation of an isometry (rigid transformation).
    This is essentially a faster version of the more general np.linalg.inv.

    Example:
    >>> T = tr.random_rotation_matrix()
    >>> T[:3, 3] = tr.random_vector(3)
    >>> T_inv = isometry_inv(T)
    >>> np.allclose(T_inv, np.linalg.inv(T))
    True
    """
    R_inv = T[:3, :3].T
    t_inv = -R_inv.dot(T[:3, 3])

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def normalized(arr):
    return arr / np.linalg.norm(arr)


def vector_projection(a, b):
    """
    Returns the vector projection of a onto b.
    https://en.wikipedia.org/wiki/Vector_projection
    """
    return a.dot(b) / b.dot(b) * b


def vector_rejection(a, b):
    """
    Returns the vector rejection of a from b. In other words,
    the orthogonal component of a that is perpendicular to b.
    https://en.wikipedia.org/wiki/Vector_projection
    """
    return a - vector_projection(a, b)


def construct_plane(dir1, dir2):
    dir1_unit = dir1 / np.linalg.norm(dir1)

    dir2_orthogonal = vector_rejection(dir2, dir1)
    dir2_orthogonal = dir2_orthogonal / np.linalg.norm(dir2_orthogonal)

    dir3 = np.cross(dir1_unit, dir2_orthogonal)

    T = np.eye(4)
    T[:3, :3] = np.array([dir1_unit, dir2_orthogonal, dir3]).T
    return T


def find_third_triangle_corner(AB, AC, BC):
    """
    Given a triangle where
      1. The lengths of each side are known,
      2. The first corner is at (0, 0)
      3. The second corner is at (AB, 0)
    Returns the position (x, y) of the third triangle.
    Note that flipping the sign of y also is a valid solution, hence both solutions are returned

           C = (x, y)
         /   \
      /        \
    A ---------- B = (AB, 0)
    
    https://math.stackexchange.com/a/544025
    """
    x = (AC**2 - BC**2 + AB**2) / (2 * AB)

    x_squared = x**2
    AC_squared = AC**2

    if x_squared > AC_squared:
        y = 0.0
    else:
        y = np.sqrt(AC_squared - x_squared)

    return np.array([x, y]), np.array([x, -y])


def find_zeros(sample_signal_up, sample_signal_down, sample_signal_x, fn):
    """
    Step 5 of Abbes and Poisson (2024).
    Given sampled dot products (e.g. Figure 7), find accurately the roots."""
    up_zeros = []
    down_zeros = []
    for i in range(1, len(sample_signal_down)):
        if np.sign(sample_signal_up[i - 1]) != np.sign(sample_signal_up[i]):
            x0 = sample_signal_x[i - 1]
            x1 = sample_signal_x[i]
            root = scipy.optimize.root_scalar(lambda q: fn(q, is_up=True), bracket=(x0, x1)).root
            up_zeros.append(root)
        if np.sign(sample_signal_down[i - 1]) != np.sign(sample_signal_down[i]):
            x0 = sample_signal_x[i - 1]
            x1 = sample_signal_x[i]
            root = scipy.optimize.root_scalar(lambda q: fn(q, is_up=False), bracket=(x0, x1)).root
            down_zeros.append(root)

    return up_zeros, down_zeros


# determine_joint_angles(O3, O4, O5, T_R0_tool, fk):


def get_dual_ik_solution(ik_solution):
    """
    Quote from Step 7:
    For CRX cobots, when a solution [J] (6 joint parameters) is valid for a Desired-Pose, it
    is easy to verify that the other corresponding solution, called the dual solution, is also valid.
    """
    J1, J2, J3, J4, J5, J6 = ik_solution
    return [
        J1 - 180,
        -J2,
        180 - J3,
        J4 - 180,
        J5,
        J6,
    ]


def harmonize_towards_zero(joint_values):
    """https://stackoverflow.com/questions/28313558/how-to-wrap-a-number-into-a-range"""
    min_val, max_val = -180, 180
    return [min_val + (x - min_val) % (max_val - min_val) for x in joint_values]
