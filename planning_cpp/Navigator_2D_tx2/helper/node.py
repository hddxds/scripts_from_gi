import numpy as np


class Node:

    def __init__(self, point):
        if len(point) != 3:
            print("Point should be in 3D space!")
            return

        self.node_x = point[0]
        self.node_y = point[1]
        self.node_z = point[2]
        self.node_cost = 0.0
        self.node_parent = None

    @property
    def point(self):
        return [self.node_x, self.node_y, self.node_z]

    @property
    def cost(self):
        return self.node_cost

    @property
    def parent(self):
        return self.node_parent