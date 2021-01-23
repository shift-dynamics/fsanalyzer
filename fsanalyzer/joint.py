import numpy as np

from .frame import Frame


class Joint:

    def __init__(self, parent, child, initial_position=0.0,
                 initial_velocity=0.0):
        pass

    def set_index(self, index):
        self.index = index

    def update(self, q, dq):
        pass


class RevoluteJoint(Joint):

    def __init__(self, parent, child, initial_position=0.0,
                 initial_velocity=0.0):
        self.index = 0
        self.parent = parent
        self.child = child
        self.initial_position = np.radians(initial_position)
        self.initial_velocity = initial_velocity
        c = np.cos(self.initial_position)
        s = np.sin(self.initial_position)
        self.child.absolute = \
            self.parent.absolute * \
            np.matrix([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # * \
        # self.parent.absolute
        print("Updating revolute joint")
        print(f"parent: {self.parent.name}")
        print(self.parent.absolute)
        print(f"child: {self.child.name}")
        print(self.child.absolute)

    def update(self, q, dq):
        c = np.cos(q[self.index])
        s = np.sin(q[self.index])
        self.child.absolute = \
            self.parent.absolute * \
            np.matrix([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # * \
        # self.parent.absolute


class PrismaticJoint(Joint):

    def __init__(self, parent, child, initial_position=0.0,
                 initial_velocity=0.0):
        self.index = 0
        self.parent = parent
        self.child = child
        self.initial_position = initial_position
        self.initial_velocity = initial_velocity
        self.child.absolute = \
            self.parent.absolute * \
            np.matrix([[1.0, 0.0, 0.0],
                       [0.0, 1.0, initial_position],
                       [0.0, 0.0, 1.0]])  # * \
        # self.parent.absolute
        print("Updating prismatic joint")
        print(f"parent: {self.parent.name}")
        print(self.parent.absolute)
        print(f"child: {self.child.name}")
        print(self.child.absolute)

    def update(self, q, dq):
        self.child.absolute = \
            self.parent.absolute * \
            np.matrix([[1.0, 0.0, 0.0],
                       [0.0, 1.0, q[self.index]],
                       [0.0, 0.0, 1.0]])  # * \
        # self.parent.absolute


class RigidJoint(Joint):
    def __init__(self, parent, child, x=0.0, y=0.0, theta=0.0):
        self.index = 0
        c = np.cos(np.radians(theta))
        s = np.sin(np.radians(theta))
        self.transform = \
            np.matrix([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])
        self.parent = parent
        self.child = child

    def update(self, q, dq):
        self.child.absolute = self.transform * self.parent.absolute
