import numpy as np
import random
from sklearn.neighbors import KDTree
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.cm as cm
import networkx as nx
import math
from shapely.geometry import Polygon, LineString
from astar2d import astar2d
import time


class RRT:

    def __init__(self):

        self.tree = nx.DiGraph()
        self.tree.add_node()

        self.pose_3d = None
        self.grid_list = []
        self.sampled_grid_kdtree = None

        self.obstacle_3d = None
        self.obstacle_kdtree = None

    def add_vertex(self, new_vertex):
        self.tree.add_node(tuple(new_vertex))

    def add_edge(self, point_1, point_2, u):
        self.tree.add_edge(tuple(point_1), tuple(point_2), orientation=u)

    @property
    def vertices(self):
        return self.tree.node()

    @property
    def edges(self):
        return self.tree.edges()

    def set_pose3d(self, pose):
        if pose is not None and len(pose) == 3:
            self.pose_3d = np.array(pose)

    def set_obstacle(self, obstacle):
        if obstacle is not None:
            self.obstacle_3d = obstacle
            self.obstacle_kdtree = KDTree(np.array(obstacle), metric='euclidean')
        else:
            print("obstacle is None, return!")

    #helper functions
    def create_grid(self, x_range=40, y_range=50, z_range=2):
        '''

           body coordinate frame used in the practical:

                            -y_range
                               ^
                  .............| -40.........
                  .            |            .
                  .            |            .
                  .            |            .
                  .            |            .
                  .            |            .
                  .            |            .
                  .            |            .
                  .            |            .
           +X <----------------0---------------- -X
               +x_range/2      0         -x_range

           '''
        x_range = range(self.pose_3d[0] - int(x_range / 2), self.pose_3d[0] + int(x_range / 2))
        y_range = range(self.pose_3d[1] - y_range, self.pose_3d[1] + 1)
        # z_range = range(self.pose_3d[2] - int(z_range / 2), self.pose_3d[2] + int(z_range / 2))
        z_range = [self.pose_3d[2]]

        grid_list = []
        for i in x_range:
            for j in y_range:
                for k in z_range:
                    p = (i, j, k)
                    grid_list.append(p)

        self.grid_list = grid_list
        self.sampled_grid_kdtree = KDTree(np.array(grid_list), metric='euclidean')


    def sample_grid(self):
        grid =