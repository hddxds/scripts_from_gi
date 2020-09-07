import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib
import matplotlib.cm as cm
from node import Node
from object3D import cuboid
from bresenham3d import Bresenham3D

from sklearn.cluster import MiniBatchKMeans, KMeans
import time

class rrt_star:

    def __init__(self, start=(0, 0, 10), target=(0, -50, 10), step_length=1, target_sample_rate=15, max_iter_num=10000):

        self.start = start
        self.target = target

        self.step_length = step_length
        self.target_sample_rate = target_sample_rate
        self.max_iter = max_iter_num

        self.obstacle_3d = None
        self.obstacle_current_height = None
        self.obstacle_neigbor_height = None
        self.obstacle_clusters = {}
        self.obstacle_polygons = {}

        self.grid_list = []
        self.node_list = []
        self.path = []
        self.minimbatchKmeans = MiniBatchKMeans(init='k-means++', n_clusters=20, batch_size=96,
                                                n_init=10, max_no_improvement=4, verbose=0)

        # for real world test
        '''
            -y
        -x     +x
            +y
        '''
        self.grid_xmin = -10
        self.grid_xmax = 10
        self.grid_ymin = 1
        self.grid_ymax = -60
        self.grid_zmin = -1
        self.grid_zmax = 10

    def find_path(self):

        self.node_list.append(Node(self.start))
        self.grid_list.append(self.start)

        iter = 0
        while self.node_distance(self.node_list[-1], Node(self.target)) > self.step_length and iter < self.max_iter:
            iter += 1
            sampled_pt = self.sample_from_grid()
            print("sample_from_grid")

            nearest_node_idx = self.getNearestPtsIndxs(sampled_pt)
            print("nearest_node_idx: ", nearest_node_idx)

            if nearest_node_idx is None:
                continue

            new_node = self.move(Node(sampled_pt), nearest_node_idx)
            print("new_node = self.move(Node(sampled_pt), nearest_node_idx)")

            if not self.collision_check(new_node):
                continue
            print("collision_check")

            near_node_idxs = self.find_near_nodes(new_node)
            print("find_near_nodes: ", near_node_idxs)

            new_node = self.find_suitable_parent(new_node, near_node_idxs)
            print("find_suitable_parent")

            if new_node.parent is not None:
                self.node_list.append(new_node)
                print("self.node_list.append(new_node): ", new_node.point)
            else:
                continue

            # self.reconnect(new_node, near_node_idxs)
            # print("self.reconnect")

        last_node_idx = self.decide_last_node()
        if not last_node_idx:
            print("Path Not Found!")
            return
        else:
            print("Path Found!")
            print("Last Node index: ", last_node_idx)
            self.path = self.get_path(last_node_idx)
            return self.path

    # for a node and a list of nodes that are close to it
    def reconnect(self, node, close_node_idxs):
        num_of_node = len(self.node_list)
        for idx in close_node_idxs:
            near_node = self.node_list[idx]
            distance = math.sqrt((near_node.x - node.x)**2
                               + (near_node.y - node.y)**2
                               + (near_node.z - node.z)**2)

            cost = distance + node.cost
            if near_node.cost > cost:
                if self.collision_check(near_node):
                    near_node.parent = num_of_node - 1
                    near_node.cost = cost

    def find_suitable_parent(self, node, close_node_idxs):
        if not close_node_idxs:
            return node

        cost_list = []
        for idx in close_node_idxs:
            potential_node = self.node_list[idx]
            distance = self.node_distance(potential_node, node)
            not_collision = self.line_collision_check(potential_node, node)
            if not_collision:
                cost_list.append(self.node_list[idx].cost + distance)
            else:
                cost_list.append(float("inf"))

        min_cost = min(cost_list)
        min_idx = close_node_idxs[cost_list.index(min_cost)]

        if min_cost == float("inf"):
            return node

        node.cost = min_cost
        node.parent = min_idx

        return node


    def find_near_nodes(self, newNode):
        num_node = len(self.node_list)
        r = 40 * math.sqrt((math.log(num_node) / num_node))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x)**2 + (node.y - newNode.y)**2
                 for node in self.node_list]
        nearinds = [dlist.index(d) for d in dlist if d <= r ** 2]
        nearinds = list(set(nearinds))
        return nearinds

    def move(self, node, nearest_node_idx):
        nearest_node = self.node_list[nearest_node_idx]
        new_node = Node(node.point)
        node_distance = np.sqrt((nearest_node.x - new_node.x)**2
                              + (nearest_node.y - new_node.y)**2
                              + (nearest_node.z - new_node.z)**2)

        if node_distance < self.step_length:
            pass
        else:
            new_node.x = nearest_node.x + (node.x - nearest_node.x) / self.step_length
            new_node.y = nearest_node.y + (node.y - nearest_node.y) / self.step_length
            new_node.z = nearest_node.z + (node.z - nearest_node.z) / self.step_length

        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def getNearestPtsIndxs(self, point):
        dlist = [(node.x - point[0])**2
                 + (node.y - point[1])**2
                 + (node.z - point[2])**2
                 for node in self.node_list if node.point != point]

        if len(dlist) < 1:
            return
        idx = dlist.index(min(dlist))
        return idx

    def sample_from_grid(self):
        if random.randint(0, 100) > self.target_sample_rate:
            # point = [random.uniform(self.grid_xmin, self.grid_xmax),
            #          random.uniform(self.grid_ymin, self.grid_ymax),
            #          random.uniform(self.grid_zmin, self.grid_zmax)]

            # print("self.grid_xmin, self.grid_xmax:",self.grid_xmin, self.grid_xmax)
            # print("self.grid_ymin, self.grid_ymax",self.grid_ymax, self.grid_ymin)
            # print("self.grid_zmin, self.grid_zmax", self.grid_zmin, self.grid_zmax)
            # print("random.randint(self.grid_xmin, self.grid_xmax):", random.randint(self.grid_xmin, self.grid_xmax))
            # print("random.randint(self.grid_ymin, self.grid_ymax):", random.randint(self.grid_ymax, self.grid_ymin))
            # print("random.randint(self.grid_zmin, self.grid_zmax):", random.randint(self.grid_zmin, self.grid_zmax))
            point = [random.randint(self.grid_xmin, self.grid_xmax),
                     random.randint(self.grid_ymax, self.grid_ymin),
                     random.randint(self.grid_zmin, self.grid_zmax)]
        else:
            point = [self.target[0], self.target[1], self.target[2]]

        return point


    def set_obstacle(self, obstalce_3d):
        self.obstacle_3d = obstalce_3d
        self.obstacle_current_height = [obstalce_3d[i] for i in range(len(obstalce_3d))
                                        if obstalce_3d[i][2] == self.start[2]]
        self.obstacle_current_height = [obstalce_3d[i] for i in range(len(obstalce_3d)) if
                                        np.abs(obstalce_3d[i][2] - self.start[2]) < 2]

        self.cluster_obstacles()

    def load_obstacle(self, obstacle_path):
        obstacle = np.load(obstacle_path)
        obstacle = np.array(obstacle['arr_0'])
        self.set_obstacle(obstacle)

        for ob in obstacle:
            print(ob)

        self.cluster_obstacles()


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


        # # step 3. merge two cuboids if 2 cuboids are close enough
        temp_polygon = cluster_polygons
        result = {}
        for i, polygon in enumerate(temp_polygon.values()):
            temp_range = range(len(cluster_polygons))
            current_label = cluster_polygons.keys()[cluster_polygons.values().index(polygon)]
            temp_range.remove(i)
            j_range = temp_range
            for j in j_range:
                if polygon.cuboid_distance(temp_polygon[j]) < 3:
                    polygon.merge_cuboid(temp_polygon[j])
                    temp_range.remove(j)

            result[current_label] = polygon

        self.obstacle_polygons = result

        return


    def visualize_world(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

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

        for idx in range(len(self.path) - 1):
            ax.plot([self.path[idx][0], self.path[idx + 1][0]],
                    [self.path[idx][1], self.path[idx + 1][1]],
                    [self.path[idx][2], self.path[idx + 1][2]],)

        ax.scatter(ox, oy, oz, c='r', s=10)
        ax.scatter(px, py, pz, c='k', s=30)
        ax.scatter(self.start[0], self.start[1], self.start[2], c='r', s=80, marker='X')
        ax.scatter(self.target[0], self.target[1], self.target[2], c='b', s=80, marker='X')

        for cuboid in self.obstacle_polygons.values():
            cuboid.draw_cuboid(ax)

        plt.savefig('world.png', dpi=800)
        plt.show()


    def collision_check(self, node, safe_margin=3):
        for obstacle_cuboid in self.obstacle_polygons.values():
            if obstacle_cuboid.in_cuboid(node.point) or obstacle_cuboid.point_distance(node.point) < safe_margin:
                return False

        return True

    '''
    given two points, construct a line with bresenham and check for collision
    with each point along the line
    '''
    def line_collision_check(self, line_start, line_end):
        line_start = (line_start.x, line_start.y, line_start.z)
        line_end = (line_end.x, line_end.y, line_end.z)
        line = Bresenham3D(line_start, line_end)
        for point in line:
            point = Node(point)
            if not self.collision_check(point):
                return False

        return True


    def node_distance(self, node_1, node_2):
        d_x = (node_1.x - node_2.x) ** 2
        d_y = (node_1.y - node_2.y) ** 2
        d_z = (node_1.z - node_2.z) ** 2
        d = np.sqrt(d_x + d_y + d_z)
        return d

    def decide_last_node(self):
        distance_to_goal = [self.node_distance(node, Node(self.target)) for node in self.node_list]
        nodes_indxs = [distance_to_goal.index(d) for d in distance_to_goal if d <= self.step_length]

        if not nodes_indxs:
            return

        min_cost = min([self.node_list[i].cost for i in nodes_indxs])
        for i in nodes_indxs:
            if self.node_list[i].cost == min_cost:
                return i


    def get_path(self, last_node_idx):
        path = [[self.target[0], self.target[1], self.target[2]]]
        print(len(self.node_list))

        last_point_idx = last_node_idx

        for node in self.node_list:
            print("node parent", node.parent)

        while self.node_list[last_point_idx].parent is not None:
            print("self.node_list[last_point_idx].parent: ", self.node_list[last_point_idx].parent)
            node = self.node_list[last_point_idx]
            print(node.point)
            path.append([node.x, node.y, node.z])
            last_point_idx = node.parent

        path.append([self.start[0], self.start[1], self.start[2]])
        print("Length of path: ", len(path))
        for p in path:
            print(p)
        return path

    # def get_path(self):
    #     # path = [[self.target[0], self.target[1], self.target[2]]]
    #     # print("get_path 1")
    #     # print(len(self.node_list))
    #     # last_point_idx = len(self.node_list) - 1
    #     # while self.node_list[last_point_idx].parent is not None:
    #     #     node = self.node_list[last_point_idx]
    #     #     print(node.point)
    #     #     path.append([node.x, node.y, node.z])
    #     #     last_point_idx = node.parent
    #     #
    #     # path.append([self.start[0], self.start[1], self.start[2]])
    #
    #     path = []
    #     for node in self.node_list:
    #         path.append(node.point)
    #     return path

if __name__ == '__main__':

    rrtstar = rrt_star(start=(0, 0, 7), target=(0, -60, 7))
    rrtstar.load_obstacle('../octomap_points3d.npz')
    s = time.time()
    path = rrtstar.find_path()
    e = time.time()
    print("Time for finding path: ", e-s)
    rrtstar.visualize_world()

