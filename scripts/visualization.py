import pandas as pd
import numpy as np
from generateSDF import readSeafloorMap
from matplotlib import pyplot as plt
from scipy import interpolate

path_fp = "../data/result.txt"

seafloor_fp = "../data/NYC/depth_grid_NYC.csv"
cell_size = 10

path_df = pd.read_csv(path_fp,skiprows=1)
path = path_df.to_numpy()
path = path[:,:3]
seafloor_mat = readSeafloorMap(seafloor_fp)
z_max = max(seafloor_mat.flatten())
z_min = min(seafloor_mat.flatten())

[w,h] = seafloor_mat.shape
w = w*cell_size
h = h*cell_size
[len,a] = path.shape
z_floor = np.zeros(len)

for i,id in enumerate(path):
    id_x = id[1]/cell_size
    id_y = id[0]/cell_size

    id_x_l = int(id_x)
    id_x_r = int(id_x) + 1
    id_y_l = int(id_y)
    id_y_r = int(id_y) + 1
    tmp_mat = np.array([[seafloor_mat[id_x_l, id_y_l], seafloor_mat[id_x_r, id_y_l]],
                        [seafloor_mat[id_x_l, id_y_r], seafloor_mat[id_x_r, id_y_r]]])
    xx, yy = np.meshgrid([id_x_l, id_x_r], [id_y_l, id_y_r])
    f = interpolate.interp2d(xx, yy, tmp_mat, kind='linear')
    z_floor[i] = f(id_x, id_y)

fig, (ax1, ax2, ax3) = plt.subplots(3,figsize=(8, 6))
fig.suptitle('Robot position(m) vs time(s)')
ax1.plot(path[:,0], label = 'x', color = "#22559c")
ax1.legend()
ax1.set_xlim([0,len])
ax1.set_ylim([0,h])
ax1.set_yticks([0,h])
ax2.set_xlim([0,len])
ax2.plot(path[:,1], label = 'y', color = "#f27370")
ax2.set_ylim([0,w])
ax2.set_yticks([0,w])
ax2.legend()
ax3.plot(path[:,2],label = 'z', color = "#76b39d")
ax3.plot(z_floor, label = 'seafloor terrain height', color = "#fdb44b")
ax3.set_xlim([0,len])
# ax3.set_ylim([-4250, -4180])
# ax3.set_yticks([-4250,-4180])
ax3.set_ylim([z_min, z_max])
ax3.set_yticks([z_min,z_max])
ax3.legend()
plt.savefig("1.png")
# plt.show()

print("Done!")