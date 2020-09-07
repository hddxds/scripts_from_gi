import numpy as np



class cuboid:

    def __init__(self, points):

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

        print(points)
        print("xmin: ", self.xmin)
        print("xmax: ", self.xmax)
        print("ymin: ", self.ymin)
        print("ymax: ", self.ymax)
        print("zmin: ", self.zmin)
        print("zmax: ", self.zmax)
        print("center: ", self.cuboid_center)


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

    @property
    def center(self):
        return self.cuboid_center

    def in_cuboid(self, point):
        if point[0] < self.xmin or point[0] > self.xmax:
            return False
        elif point[1] < self.ymin or point[1] > self.ymax:
            return False
        elif point[2] < self.zmin or point[2] > self.zmax:
            return False

        return True

    def point_distance(self, point):
        if self.in_cuboid(point):
            return
        else:
            point_center_distance = np.linalg.norm(self.center - point)
            z_distance = np.abs(self.center[2] - point[2])
            horizontal_distance = np.sqrt(point_center_distance**2 - z_distance**2)
            min_distance = horizontal_distance - max(self.xsize, self.ysize) / 2.0
            return min_distance

    def cuboid_distance(self, cuboid):
        pass


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

if __name__ == '__main__':

    ps = [[0, 0, 0],
          [3, 3, 3]]

    c = cuboid(ps)
    c.point_distance()