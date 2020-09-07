import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def save_points3D(points_set, save_path="./octomap_points3d.npz"):

    print("save points 3D started")
    points_array = np.array(list(points_set))
    np.savez(save_path, points_array)
    print("save points 3D finished")

def load_points3D(load_path="./octomap_points3d.npz"):
    npzfile = np.load(load_path)
    points = npzfile['arr_0']

    return points

def visualize_points(points):
    ax = plt.gca(projection="3d")
    x = []
    y = []
    z = []
    for point in points:
        print(point)
        x.append(float(point[0]))
        y.append(float(point[1]))
        z.append(float(point[2]))

    body = []
    X = 6
    Y = 6
    Z = 3
    for i in range(-X, X):
        for j in range(-Y, Y):
            for k in range(-Z, Z):
                body.append((i, j, k))

    for point in body:
        x.append(float(point[0]))
        y.append(float(point[1]))
        z.append(float(point[2]))

    ax.scatter(x, y, z, c='r', s=5)

    plt.show()
    plt.savefig('octomap_center_points.png')

if __name__ == '__main__':

    npz_points = load_points3D()
    print(npz_points)

    visualize_points(npz_points)