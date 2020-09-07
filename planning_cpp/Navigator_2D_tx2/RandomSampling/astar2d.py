from queue import PriorityQueue
import numpy as np

def heuristic(n1, n2):
    return np.linalg.norm(np.array(n2) - np.array(n1))

def astar2d(graph, start, goal):

    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    current_node = start
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
        # if current_node == goal or heuristic(current_node, goal) < 10:
            print("Current node is: ", current_node)
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)

    path = []
    path_cost = 0
    if found:
        print("current node: ", current_node)

        # retrace steps
        path = []
        n = current_node
        path.append(n)

        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]

        path.append(branch[n][1])

    return path[::-1], path_cost