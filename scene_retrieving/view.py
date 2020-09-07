import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file = open('/home/shr/software/GAAS_scene_retrieve/GAAS/algorithms/scene_retrieving/ALL_T.txt').readlines()

old_Ts = []
relative_Ts = []
for line in file:
    line = line.split(',')
    old_t = np.array([float(line[0]), float(line[1]), float(line[2]) ])
    relative_t = np.array([float(line[3]), float(line[4]), float(line[5].split('\n')[0]) ])

    if(sum(old_t) > 1e3 or sum(relative_t) > 1e3):
        continue

    old_Ts.append(old_t)
    relative_Ts.append(relative_t)
    print(old_t, relative_t)


old_x = []
old_y = []
old_z = []
for pose in old_Ts:
    old_x.append(float(pose[0]))
    old_y.append(float(pose[1]))
    old_z.append(float(pose[2]))

fig = plt.figure()
ax = fig.add_subplot(211, projection='3d')
ax.scatter3D(old_x, old_y, old_z, c='r')

# new_T = []
# for idx, relative_t in enumerate(relative_Ts):
#     result_t = np.matmul(old_Ts[idx], relative_t)
#     new_T.append(relative_t)

new_T = []
for idx, relative_t in enumerate(relative_Ts):
    print relative_t
    #result_t = old_Ts[idx] + [relative_t[0], -relative_t[2], relative_t[1]]
    result_t = old_Ts[idx] + [relative_t[1], relative_t[2], relative_t[0]]
    new_T.append(result_t)

new_x = []
new_y = []
new_z = []
for pose in new_T:
    new_x.append(pose[0])
    new_y.append(pose[1])
    new_z.append(pose[2])


ax = fig.add_subplot(212, projection='3d')
ax.scatter3D(new_x, new_y, new_z, c='r')
plt.show()


relative_x = []
relative_y = []
relative_z = []
for pose in relative_Ts:
    relative_x.append(pose[0])
    relative_y.append(pose[1])
    relative_z.append(pose[2])

relative_x = [x for x in relative_x if x < 1e3]
relative_y = [x for x in relative_y if x < 1e3]
relative_z = [x for x in relative_z if x < 1e3]

# plt.plot(relative_x, c='r')
# plt.plot(relative_y, c='g')
# plt.plot(relative_z, c='b')
# plt.show()
