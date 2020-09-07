import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib
import matplotlib.cm as cm
import time
from sklearn.cluster import MiniBatchKMeans, KMeans
from object3D import cuboid

class PotentialField3D:

    def __init__(self):
        self.obstacle_3d = None
        self.obstacle_current_height = None

        self.grid = None
        self.grid_list = None
        self.grid_dict = {}

        self.pose_3d = None
        self.target_3d = None
        self.forces_3d = None

        self.path = None
        self.pmap = None

        self.alpha = 1
        self.beta = 30
        self.q_max = 5

        self.movement_list = self.generate_movementlist()
        self.minimbatchKmeans = MiniBatchKMeans(init='k-means++', n_clusters=15, batch_size=96,
                                                n_init=10, max_no_improvement=4, verbose=0)
        self.obstacle_clusters = {}
        self.obstacle_polygons = {}

        # for test
        self.p1 = []
        self.p2 = []

    def generate_movementlist(self):
        movement_list = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    movement_list.append([i, j, k])

        return movement_list

    def find_path(self, start, target):
        self.pose_3d = start
        self.target_3d = target
        self.create_grid()

        if start is None:
            print("start position is None, return.")
            return

        if target is None:
            print("target position is None, return.")
            return

        if self.obstacle_3d is None:
            print("obstacle is None, return.")
            return

        if self.grid is None:
            print("grid is None, return.")
            return

        # step 1, iterate points and calculate force(sum of attraction and repulsion) for each point
        s = time.time()
        x_range = range(self.pose_3d[0] - int(self.grid.shape[0]/2), self.pose_3d[0] + int(self.grid.shape[0]/2))
        y_range = range(self.pose_3d[1] - self.grid.shape[1], self.pose_3d[1] + 1)
        z_range = range(self.pose_3d[2] - int(self.grid.shape[2]/2), self.pose_3d[2] + int(self.grid.shape[2]/2))
        #z_range = [self.pose_3d[2]]

        grid_list = []
        forces = []
        grid_dict = {}
        p1 = []
        p2 = []
        for i in x_range:
            for j in y_range:
                for k in z_range:
                    p = (i, j, k)

                    # s = time.time()
                    force = self.attraction(p, self.target_3d)

                    for obs_polygon in self.obstacle_polygons.values():

                        if obs_polygon.in_cuboid(p):
                            p1.append(p)
                            force += np.abs(self.repulsion(p, obs_polygon.center))


                        if obs_polygon.point_distance(p) <= self.q_max:
                            p2.append(p)
                            force += np.abs(self.repulsion(p, obs_polygon.center))

                    grid_list.append(p)
                    forces.append(force)
                    grid_dict[p] = force
                    # e = time.time()
                    # print("TIme taken for calculating force: ", e-s)

        # for test:
        self.p1 = p1
        self.p2 = p2

        print("grid list size: ", len(grid_list))
        self.grid_list = grid_list
        self.forces_3d = forces
        self.grid_dict = grid_dict
        e = time.time()
        print("Time taken for generating grid and computing force: ", e-s)

        # step 2, iterate point and find its neighbor with lowest force
        s = time.time()
        current_pose = self.pose_3d
        next_pose = current_pose
        path = []

        while np.linalg.norm(np.array(next_pose) - self.target_3d) > 2 and len(path) < 500:

            next_poses = [(next_pose + movement) for movement in self.movement_list if tuple((next_pose + movement)) in self.grid_dict]

            # print(next_poses)

            if len(next_poses) < 1:
                print("Path Not found!")
                return

            next_forces = [self.grid_dict[tuple(next_pose)] for next_pose in next_poses]
            min_index = next_forces.index(min(next_forces))
            next_pose = next_poses[min_index]

            path.append(next_pose)

        self.path = path
        e = time.time()
        print("Time taken for finding path: ", e-s)
        return path


    # helper functions
    def visualize_world(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        skip_size = 1
        x = []
        y = []
        z = []
        forces = []
        for idx, p in enumerate(self.grid_list):
            if idx % 1 == 0:
                x.append(p[0])
                y.append(p[1])
                z.append(p[2])
                forces.append(self.grid_dict[p])
                # print("self.grid_dict[p]:", p, self.grid_dict[p])

        ox = []
        oy = []
        oz = []
        for idx, obstacle in enumerate(self.obstacle_3d):
                ox.append(obstacle[0])
                oy.append(obstacle[1])
                oz.append(obstacle[2])

        px = []
        py = []
        pz = []
        for idx, path_point in enumerate(self.path):
            px.append(path_point[0])
            py.append(path_point[1])
            pz.append(path_point[2])


        p1x = []
        p1y = []
        p1z = []
        for idx, path_point in enumerate(self.p1):
            p1x.append(path_point[0])
            p1y.append(path_point[1])
            p1z.append(path_point[2])

        p2x = []
        p2y = []
        p2z = []
        for idx, path_point in enumerate(self.p2):
            p2x.append(path_point[0])
            p2y.append(path_point[1])
            p2z.append(path_point[2])

        for cuboid in self.obstacle_polygons.values():
            cuboid.draw_cuboid(ax)

        ax.scatter(self.pose_3d[0], self.pose_3d[1], self.pose_3d[2], c='r', s=80, marker='X')
        ax.scatter(self.target_3d[0], self.target_3d[1], self.target_3d[2], c='b', s=80, marker='X')

        # ax.scatter(x, y, z, c=forces, s=2)
        ax.scatter(ox, oy, oz, c='r', s=10)
        ax.scatter(px, py, pz, c='k', s=10)

        # for testing
        # ax.scatter(p1x, p1y, p1z, c='r', s=10)
        # ax.scatter(p2x, p2y, p2z, c='g', s=10)

        plt.savefig('world.png', dpi=800)
        plt.show()

    def attraction(self, position, goal):
        return self.alpha * np.linalg.norm(np.array(position) - np.array(goal))


    def repulsion(self, position, obstacle):
        position = np.array(position)
        obstacle = np.array(obstacle)

        return self.beta\
            * ((1 / self.q_max) - (1 / np.linalg.norm(position - obstacle)))\
            * (1 / np.linalg.norm(position - obstacle) ** 2)


    '''

       body coordinate frame used in the practical:

                           -Y
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
              +20          0         -20
       '''
    def create_grid(self):
        x_size = 40
        y_size = 60
        z_size = 20

        grid = np.zeros((x_size, y_size, z_size))

        print("grid generated, grid shape is: ", np.shape(grid))
        self.grid = grid
        return self.grid


    # load obstacle 3D points from npz
    def load_obstacle(self, obstacle_path):
        obstacle = np.load(obstacle_path)
        self.obstacle_3d = np.array(obstacle['arr_0'])

        # for ob in self.obstacle_3d:
        #     print(ob)

    def set_obstacle(self, obstacle):
        if len(obstacle) < 1:
            print("setting obstacle failed, number of points less than 1.")
            return
        self.obstacle_3d = obstacle

    # return a 2D obstacle map
    def get_obstacle_at_current_height(self):
        obs = []

        for ob in self.obstacle_3d:
            if ob[2] == self.pose_3d[2]:
                obs.append(ob[0:2])

        if len(obs) < 1:
            return

        self.obstacle_current_height = obs

        return obs


    def cluster_obstacles(self):

        # step 1. use Kmeans to cluster
        s = time.time()
        self.minimbatchKmeans.fit(self.obstacle_3d)
        results = self.minimbatchKmeans.fit_predict(self.obstacle_3d)

        clustered_obs = {}
        for idx, ob in enumerate(self.obstacle_3d):
            if results[idx] not in clustered_obs:
                clustered_obs[results[idx]] = []
                clustered_obs[results[idx]].append(ob)
            else:
                clustered_obs[results[idx]].append(ob)

        self.obstacle_clusters = clustered_obs
        e = time.time()
        print("time taken for Kmeans:", e-s)


        # step 2. for each cluster, construct a cuboid
        cluster_polygons = {}
        for label in self.obstacle_clusters.keys():
            points = self.obstacle_clusters[label]
            cube = cuboid(points)
            cluster_polygons[label] = cube

        self.obstacle_polygons = cluster_polygons



if __name__ == '__main__':

    pf = PotentialField3D()
    pf.load_obstacle('../octomap_points3d.npz')
    pf.cluster_obstacles()

    s = time.time()
    pf.find_path(np.array([0, 0, 5]), np.array([-30, -50, 10]))
    e = time.time()
    print("Time taken for all: ", e-s)

    pf.visualize_world()