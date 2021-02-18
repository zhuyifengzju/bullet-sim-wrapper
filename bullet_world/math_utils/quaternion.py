"""3D orientation as quaternions.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

from bullet_world.transformations import euler_from_quaternion
from bullet_world.transformations import matrix3_from_quaternion
from bullet_world.math_utils.euler import Euler

class Quaternion(np.ndarray):
    """Quaternion.
    """

    def __new__(cls, value):
        """Initialize the orientation.

        Args:
            value: A 4-dimensional float32 numpy array of [x, y, z, w].
        """
        obj = np.asarray(value).reshape(4,).view(cls)
        return obj

    def __str__(self):
        return '[%g, %g, %g, %g]' % (self.x, self.y, self.z, self.w)

    def to_wxyz(self):
        return self.quaternion_wxyz

    def to_xyzw(self):
        return self.quaternion_xyzw
    
    
    @property
    def euler(self):
        return Euler(euler_from_quaternion(self))

    @property
    def quaternion_xyzw(self):
        return self

    @property
    def quaternion_wxyz(self):
        return np.array([self[3], self[0], self[1], self[2]])        
        
    @property
    def matrix3(self):
        return matrix3_from_quaternion(self)

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def z(self):
        return self[2]

    @property
    def w(self):
        return self[3]

    @x.setter
    def x(self, value):
        self[0] = value

    @y.setter
    def y(self, value):
        self[1] = value

    @z.setter
    def z(self, value):
        self[2] = value

    @w.setter
    def w(self, value):
        self[3] = value
