import numpy as np
import matplotlib.pyplot as plt


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


class Astar:

    def __init__(self):
        self.pose_3d = [0, 0, 0]
        self.target_2d = [0, 0]
        self.pose_2d = None

        self.obstacle_3d = None
        self.obstacle_2d = None

        self.grid = None

    def set_obstacle(self, obstacle3d):
        self.obstacle_3d = np.array(list(obstacle3d))
        self.set_obstacle_at_current_height()

    def set_pose_d(self, pose_d):
        self.pose_3d = pose_d
        self.pose_2d = pose_d[0:2]

    def set_obstacle_at_current_height(self):
        obs = []

        print(self.obstacle_3d)
        print(self.pose_3d)

        if len(self.pose_3d) is not 3:
            return

        for ob in self.obstacle_3d:
            if ob[2] == self.pose_3d[2]:
                obs.append(ob[0:2])

        if len(obs) < 1:
            return

        self.obstacle_2d = obs

        return self.obstacle_2d

    def create_grid(self):
        min_y = self.pose_3d[0] - 20
        max_y = self.pose_3d[0] + 20

        min_x = self.pose_3d[1] - 40
        max_x = self.pose_3d[1]

        xs = range(min_x, max_x)
        ys = range(min_y, max_y)

        grid = []
        for x in ys:
            for y in xs:
                grid.append([x, y])

        grid = np.zeros((40, 40))
        print(grid)

        self.grid = grid

        for ob in self.obstacle_2d:
            self.grid[ob[0], ob[1]] = 1

        return self.grid

    def load_obstacle(self, obstacle_path):
        obstacle = np.load(obstacle_path)
        obs_3d = np.array(obstacle['arr_0'])

        return obs_3d

    def save_grid_to_image(self):
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        plt.xlabel('X')
        plt.ylabel('Y')

        # plot obstacles in red
        x = []
        y = []
        for p in self.obstacle_2d:
            x.append(p[0])
            y.append(p[1])

        # plot free space
        obs_set = set(self.obstacle_2d)
        world_grid = set(self.grid)
        free_space = world_grid - obs_set

        x = []
        y = []
        for p in free_space:
            x.append(p[0])
            y.append(p[1])

        ax1.scatter(x, y, c='r', marker='o')

        # start and end position
        start = self.pose_d
        end = self.target_d
        ax1.scatter(start[0], start[1], s=100,  c='r', marker="s")
        ax1.scatter(end[0], end[1], s=100, c='r', marker="s")

        plt.savefig("world.png")

    def find_path(self, start, end):

        self.pose_d = start
        self.target_d = end

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1),
                                 (1, 1)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(self.grid) - 1) or node_position[0] < 0 or node_position[1] > (
                        len(self.grid[len(self.grid) - 1]) - 1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if self.grid[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                            (child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)






if __name__ == '__main__':


    # maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    #
    # start = (0, 0)
    # end = (0, 6)
    #
    # astar = Astar()
    # path = astar.find_path(maze, start, end)
    # print(path)


    test = Astar()
    obs_3d = test.load_obstacle('../octomap_points3d.npz')
    test.set_pose_d([0, 0, 10])
    test.set_obstacle(obs_3d)
    test.create_grid()

