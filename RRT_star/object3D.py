import numpy as np
from sklearn.cluster import MiniBatchKMeans, KMeans


class cuboid:

    def __init__(self, points):

        self.initialize_cuboid(points)

        # print(points)
        # print("xmin: ", self.xmin)
        # print("xmax: ", self.xmax)
        # print("ymin: ", self.ymin)
        # print("ymax: ", self.ymax)
        # print("zmin: ", self.zmin)
        # print("zmax: ", self.zmax)
        # print("center: ", self.cuboid_center)


    '''
    compute cuboid parameters from a set of points
    '''
    def initialize_cuboid(self, points):

        self.xmin = min([point[0] for point in points])
        self.xmax = max([point[0] for point in points])
        self.ymin = min([point[1] for point in points])
        self.ymax = max([point[1] for point in points])
        self.zmin = min([point[2] for point in points])
        self.zmax = max([point[2] for point in points])

        self.xsize = self.xmax - self.xmin
        self.ysize = self.ymax - self.ymin
        self.zsize = self.zmax - self.zmin

        self.cuboid_center = np.array(((self.xmin + self.xmax) / 2.0,
                                       (self.ymin + self.ymax) / 2.0,
                                       (self.zmin + self.zmax) / 2.0))

        self.cuboid_points = points


    '''
    if a point is in cuboid
    '''
    def in_cuboid(self, point):
        if point[0] < self.xmin or point[0] > self.xmax:
            return False
        elif point[1] < self.ymin or point[1] > self.ymax:
            return False
        elif point[2] < self.zmin or point[2] > self.zmax:
            return False

        return True

    '''
    shortest path from point to cuboid
    '''
    def point_distance(self, point):
        if self.in_cuboid(point):
            return
        else:
            dx = max(self.xmin - point[0], 0, point[0] - self.xmax)
            dy = max(self.ymin - point[1], 0, point[1] - self.ymax)
            dz = max(self.zmin - point[2], 0, point[2] - self.zmax)
            return np.sqrt(dx*dx + dy*dy + dz*dz)

    '''
    return the shortest distance from self to another cuboid
    '''
    # def cuboid_distance(self, cuboid):
    #     pass


    '''
    naive method to measure the shortest distance between two cuboids
    '''
    def cuboid_distance(self, target_cuboid):
        if not isinstance(target_cuboid, cuboid):
            return

        min_vertex_distance = float("inf")
        for vertex_target in target_cuboid.vertices:
            for vertex_current in self.vertices:

                if self.in_cuboid(vertex_target):
                    return 0

                distance = np.linalg.norm(np.array(vertex_current)-np.array(vertex_target))
                if distance < min_vertex_distance:
                    min_vertex_distance = distance

        return min_vertex_distance

    '''
    merge with another cuboid, use current label
    '''
    def merge_cuboid(self, target_cuboid):
        if not isinstance(target_cuboid, cuboid):
            print("Not cuboid, remain the same!")
            return

        cuboid_points = self.cuboid_points + target_cuboid.cuboid_points

        self.initialize_cuboid(cuboid_points)


    # you should input a plt instance
    def draw_cuboid(self, ax):
        vertices = self.vertices

        for i in range(7):
            if i == 3:
                continue
            ax.plot([vertices[i][0], vertices[i+1][0]],
                    [vertices[i][1], vertices[i+1][1]],
                    [vertices[i][2], vertices[i+1][2]])

        for i in range(4):
            ax.plot([vertices[i][0], vertices[i+4][0]],
                    [vertices[i][1], vertices[i+4][1]],
                    [vertices[i][2], vertices[i+4][2]])

        ax.plot([vertices[3][0], vertices[0][0]],
                [vertices[3][1], vertices[0][1]],
                [vertices[3][2], vertices[0][2]])
        ax.plot([vertices[7][0], vertices[4][0]],
                [vertices[7][1], vertices[4][1]],
                [vertices[7][2], vertices[4][2]])

        return ax

    @property
    def center(self):
        return self.cuboid_center

    @property
    def points(self):
        return self.cuboid_points

    # @property
    # def label(self):
    #     return self.cuboid_label

    @property
    def vertices(self):

        '''
         p8                 p7
         /----------------/|
      p5/_|___________p6_/ |
        | |             |  |
        | /p4-----------|-/ p3
        |/______________|/
        p1              p2
        '''

        vertices = []

        vertices.append([self.xmin, self.ymin, self.zmin])
        vertices.append([self.xmax, self.ymin, self.zmin])
        vertices.append([self.xmax, self.ymax, self.zmin])
        vertices.append([self.xmin, self.ymax, self.zmin])

        vertices.append([self.xmin, self.ymin, self.zmax])
        vertices.append([self.xmax, self.ymin, self.zmax])
        vertices.append([self.xmax, self.ymax, self.zmax])
        vertices.append([self.xmin, self.ymax, self.zmax])

        return vertices

if __name__ == '__main__':

    ps = [[0, 0, 0],
          [3, 3, 3]]

    c = cuboid(ps)
    c.point_distance([1, 2, 3])

    ps2 = [[1, 2, 3],
          [4, 5, 6]]

    c2 = cuboid(ps2)
    distance = c.cuboid_distance(c2)
    print("Min distance is: ", distance)

    c.merge_cuboid(c2)
    # print(c.label)
    print("c points: ", c.points)

    c3 = c
    print("c3 points: ", c3.points)