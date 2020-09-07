import numpy as np


class Node:

    def __init__(self, point):
        if len(point) != 3:
            print("Point should be in 3D space!")
            return

        self.x = point[0]
        self.y = point[1]
        self.z = point[2]
        self.cost = 0.0
        self.parent = None

    @property
    def point(self):
        return [self.x, self.y, self.z]

