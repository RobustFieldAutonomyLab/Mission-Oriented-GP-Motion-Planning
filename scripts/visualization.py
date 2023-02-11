import pandas as pd
import numpy as np
from generateSDF import readSeafloorMap
from matplotlib import pyplot as plt

path_fp = "../data/result.txt"

seafloor_fp = "../data/lower_bay/depth_grid_lower_bay.csv"

path_df = pd.read_csv(path_fp,skiprows=1)
path = path_df.to_numpy()
path = path[:,:3]
seafloor_mat = readSeafloorMap(seafloor_fp)
z_max = max(seafloor_mat.flatten())
z_min = min(seafloor_mat.flatten())

[w,h] = seafloor_mat.shape
[len,a] = path.shape
id_x = path[:,0]
id_x = id_x.astype(int)
id_y = path[:,1]
id_y = id_y.astype(int)
z_floor = seafloor_mat[id_y, id_x]
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