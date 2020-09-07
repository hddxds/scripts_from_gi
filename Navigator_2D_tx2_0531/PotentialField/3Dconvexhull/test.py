import  numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

points = [[0, 0, 0],
          [2, 0, 0],
          [2, 2, 0],
          [0, 2, 0],
          [0, 0, 2],
          [2, 0, 2],
          [2, 2, 2],
          [0, 2, 2],
          [1, 1, 1]]

hull = ConvexHull(points)

x = []
y = []
z = []
for simplex in hull.simplices:
    x.append(simplex[0])
    y.append(simplex[1])
    z.append(simplex[2])



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
plt.show()


# x = []
# y = []
# z = []
# for v in hull.vertices:
#     print(v)
#     # x.append(v[0])
#     # y.append(v[1])
#     # z.append(v[2])
#
#
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x, y, z)
# plt.show()