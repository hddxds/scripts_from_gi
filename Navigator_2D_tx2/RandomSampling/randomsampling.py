import numpy as np
import random
from sklearn.neighbors import KDTree
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import networkx as nx
import math
from shapely.geometry import Polygon, LineString
from astar2d import astar2d
import time

'''
This class use random sampling together with KD tree to find a path.
Steps are:
1. randomly sample 100 points in the target drone body space, ie a 5x2x5 meter space with the drone in the origin;
2. connect the nodes and eliminate lines that are too close or interset with obstacles;
3. use KD tree to retrieve the path
'''

class randomsampling:

    '''
    For a drone, construct a space of size w x h x d meters:

      --------------------
     /                   /|  h
    /                   / /
    -------------------- /  d
    |      (drone)     |/
    --------------------
            w

    1. Randomly sample some points in the given space;
    2. connect each points with one another in a radius via KDtree;
    3. use KNN to cluster obstacle points and represent them by Shapely
    4. for connected lines, see if they intersect with clustered obstacles;
    5. remove intersected lines and only keep free ones;
    6. given filtered lines, use A* to find a path and return it.

    Note: obstacles and drone position are in discrete grids.
    '''

    '''
    grid size must be equal to the one defined in Navigator
    '''
    def __init__(self, w=8, h=1, d=8, grid_size=0.1):
        self.grid_size = grid_size
        self.w = int(w / self.grid_size)  # should be odd number
        self.h = int(h)
        self.d = int(d / self.grid_size)

        self.pose_d = None
        self.target_d = None
        self.drone_grid = np.zeros((self.w, self.d))

        self.obstacle = None
        self.obstacle_current_height = None
        self.grid = None
        self.kdtree = None
        self.free_space_kdtree = None

        self.clustered_obstacles = {}
        self.clustered_obstacles_polygon = []

    def find_path(self, pose_d, target_pose_d, obstacle_3d):
        self.target_d = target_pose_d[0:2]
        pose_d = (0, 0, pose_d[2])

        if len(pose_d) is not 3 or len(target_pose_d) is not 3:
            print("Drone current pose and target pose should be in 3D grid, of length 3")
            return

        if pose_d == target_pose_d:
            print("Start pose and target pose shouldn't be the same!")
            return None, None

        self.set_pose_d(pose_d)

        t1 = time.time()
        self.create_grid()
        t2 = time.time()
        print("time taken for create grid: ", t2-t1)

        t1 = time.time()
        self.set_obstacle(obstacle_3d)
        t2 = time.time()
        print("time taken for set_obstacle: ", t2 - t1)

        t1 = time.time()
        obs_2d = self.get_obstacle_at_current_height()
        t2 = time.time()
        print("time taken for get_obstacle_at_current_height: ", t2-t1)

        if obs_2d is None:
            print("Obstacle is empty!")
            return None, None

        t1 = time.time()
        self.obstacle_cluster(number_of_cluster=30)
        t2 = time.time()
        print("time taken for obstacle_cluster: ", t2 - t1)

        t1 = time.time()
        samples = self.random_samples(number=200)
        t2 = time.time()
        print("time taken for random_samples: ", t2 - t1)

        t1 = time.time()
        graph = self.connet_nodes(samples, radius=25)
        t2 = time.time()
        print("time taken for connet_nodes: ", t2 - t1)


        # idss = list(self.free_space_kdtree.query_radius(np.array([target_pose_d[0],
        #                                                           target_pose_d[1]]).reshape(1, -1),
        #                                                 r=6))[0]

        # if len(idss) < 1:
        #     print("No Target Found Near Target Position!")
        #     return
        #
        # target_pose_d = samples[idss[0]]
        target_pose_d = target_pose_d[0:2]

        print("Nearest target is: ", target_pose_d)
        t1 = time.time()
        path_2d, _ = astar2d(graph, pose_d[0:2], target_pose_d)
        t2 = time.time()
        print("time taken for astar2d: ", t2 - t1)

        self.save_world_to_img(pose_d, target_pose_d, obs_2d, samples, graph, path_2d, "world_no_path.png")

        print(pose_d[0:2], target_pose_d[0:2])
        print("find_path result: ", path_2d)

        if len(path_2d) < 1:
            print("No path found!, len(path_2d) < 1, path: ", path_2d)
            return None, None

        self.save_world_to_img(pose_d, target_pose_d, obs_2d, samples, graph, path_2d)

        t1 = time.time()
        path_3d = []
        for wp in path_2d:
            wp = list(wp)
            wp.append(self.pose_d[2])
            wp = np.array(wp)
            path_3d.append(wp)

        t2 = time.time()
        print("time taken for 3d points: ", t2 - t1)

        print("path_3d: ", path_3d)

        return path_2d, path_3d

    def save_world_to_img(self, start, end, obstacles, samples, graph, path, name="world_path.png"):
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        plt.xlabel('X')
        plt.ylabel('Y')

        # plot obstacles in red
        x = []
        y = []
        for p in obstacles:
            x.append(p[0])
            y.append(p[1])

        ax1.scatter(x, y, c='r', marker='o')

        # plot sampled points in green
        x_s = []
        y_s = []
        for s in samples:
            x_s.append(s[0])
            y_s.append(s[1])

        ax1.scatter(x_s, y_s, c='g', marker='D')

        # start and end position
        ax1.scatter(start[0], start[1], s=100,  c='r', marker="s")
        ax1.scatter(end[0], end[1], s=100, c='r', marker="s")

        # plot graph edges in black
        for (n1, n2) in graph.edges:
            plt.plot([n1[0], n2[0]], [n1[1], n2[1]], 'black', alpha=0.5)

        # plot path
        path_pairs = zip(path[:-1], path[1:])

        for (n1, n2) in path_pairs:
            plt.plot([n1[0], n2[0]], [n1[1], n2[1]], 'red', linewidth=3)

        plt.savefig(name)

    def set_pose_d(self, pose):
        self.pose_d = pose

    def sample_points(self):
        pass

    # given current drone pose in discrete, return obstacle list at the same height
    def set_obstacle(self, obstacle):
        self.obstacle = np.array(list(obstacle))

    # return a 2D obstacle map
    def get_obstacle_at_current_height(self):
        obs = []
        #print(self.obstacle)
        # print("pose d:", self.pose_d)
        # print("self.obstacle:", self.obstacle)
        # print("type of self.obstacle: ", type(self.obstacle))
        # print("length of self.obstacle: ", len(self.obstacle))

        for ob in self.obstacle:
            if ob[2] == self.pose_d[2]:
                obs.append(ob[0:2])
                # print(ob[0:2])

        if len(obs) < 1:
            return

        # print("points used in KDTree: ", len(obs), obs)
        self.kdtree = KDTree(obs, metric='euclidean')
        self.obstacle_current_height = obs
        self.obstacle_cluster()

        return obs

    def load_obstacle(self, obstacle_path):
        obstacle = np.load(obstacle_path)
        self.obstacle = np.array(obstacle['arr_0'])


    def create_grid(self):

        min_y = self.pose_d[0] - 20
        max_y = self.pose_d[0] + 20

        min_x = self.pose_d[1] - 40
        max_x = self.pose_d[1]

        xs = range(min_x, max_x)
        ys = range(min_y, max_y)

        grid = []
        for x in ys:
            for y in xs:
                grid.append([x, y])

        self.grid = grid

        return self.grid

    # randomly sample given number of points in the grid and return sampled points
    def random_samples(self, number=200):

        # step 1, create samples
        samples = random.sample(self.grid, number)

        # step 2, collision detection use kdtree
        sample_b = []
        for sample in samples:
            idss = list(self.kdtree.query_radius(np.array([sample[0], sample[1]]).reshape(1, -1), r=3))[0]

            if len(idss) > 0:
                pass
            else:
                sample_b.append(sample)

        sample_b.append((0, 0))
        sample_b.append((0, -30))

        print("Number of Sampled Points: ", len(sample_b))

        self.free_space_kdtree = KDTree(sample_b, metric='euclidean')

        return sample_b

    def obstacle_cluster(self, number_of_cluster=10):
        kmeans = KMeans(n_clusters=number_of_cluster)
        #print("self.obstacle: ", self.obstacle_current_height)

        kmeans.fit(self.obstacle_current_height)
        #print("cluster centers", kmeans.cluster_centers_)

        y_lables = kmeans.fit_predict(self.obstacle_current_height)
        #print(y_lables)

        for idx, y_lable in enumerate(y_lables):
            #print(idx, y_lable, self.obstacle_current_height[idx])

            if y_lable not in self.clustered_obstacles.keys():
                self.clustered_obstacles[y_lable] = []
                self.clustered_obstacles[y_lable].append(self.obstacle_current_height[idx])
            else:
                self.clustered_obstacles[y_lable].append(self.obstacle_current_height[idx])

        for key in self.clustered_obstacles.keys():

            if len(self.clustered_obstacles[key]) < 3:
                continue

            plg = Polygon(self.clustered_obstacles[key])
            self.clustered_obstacles_polygon.append(plg)

        #print("plgs:", self.clustered_obstacles_polygon)


    # construct a graph from a list of nodes (results of random_samples)
    def connet_nodes(self, nodes, radius):

        tree = KDTree(nodes)
        graph = nx.Graph()

        print("Number of Nodes: ", len(nodes))

        for node in nodes:
            ids = tree.query([node], radius, return_distance=False)[0]

            for id in ids:
                node_2 = nodes[id]

                if node_2 == node:
                    continue

                if not self.line_interset_obstacle((node, node_2)):
                    graph.add_edge(tuple(node), tuple(node_2), weight=1)

                # graph.add_edge(tuple(node), tuple(node_2), weight=1)

        graph.remove_nodes_from(nx.isolates(graph))
        for component in list(nx.connected_components(graph)):
            if len(component) < 4:
                for node in component:
                    graph.remove_node(node)

        return graph


    def line_interset_obstacle(self, line):
        ls = LineString([line[0], line[1]])

        for polygon in self.clustered_obstacles_polygon:
            if ls.intersects(polygon):
                return True

        return False

if __name__ == '__main__':

    rs = randomsampling()
    rs.load_obstacle('../octomap_points3d.npz')

    rs2 = randomsampling()
    path = rs2.find_path((0, 0, 10), (0, -30, 10), rs.obstacle)

    plt.show()

