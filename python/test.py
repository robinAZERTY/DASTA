import navpy as nav
import numpy as np
# rotate a 3d vector by a quaternion
def quatrot(q, v):
    q = q / np.linalg.norm(q)
    qvec = q[1:]
    uv = np.cross(qvec, v)
    uuv = np.cross(qvec, uv)
    return v + 2 * (q[0] * uv + uuv)

q = np.array([0.5, 0.5, 0.5, 0.5])
v = np.array([1, 0, 0])
print(quatrot(q, v))
print(nav.quat2dcm(q))# to get the rotation matrix