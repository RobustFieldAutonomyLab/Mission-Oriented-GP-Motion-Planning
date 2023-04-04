import sys, getopt
import pandas as pd
import numpy as np
from generateSDF import readSeafloorMap
from matplotlib import pyplot as plt
from scipy import interpolate

def main(argv):
    input_traj_file = ''
    input_map_file = ''
    output_file = '1.png'
    cell_size = 1

    options = "h:t:m:o:c:"
    long_options = ["Help", "Input_trajectory", "Input_map", "Output", "Cell_size"]

    opts, args = getopt.getopt(argv, options, long_options)
    for opt, arg in opts:
        if opt in ("-h", "--Help"):
            print('test.py -t <input trajectory file> -o <output image file> -c <cell size> -m <input map file>')
            sys.exit()
        elif opt in ("-t", "--Input_trajectory"):
            input_traj_file = arg
        elif opt in ("-o", "--Output"):
            output_file = arg
        elif opt in ("-c", "--Cell_size"):
            cell_size = float(arg)
        elif opt in ("-m", "--Input_map"):
            input_map_file = arg

    draw_path(input_traj_file, input_map_file, cell_size, output_file)

def draw_path(path_fp, seafloor_fp, cell_size, file_path):
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

    # fig, (ax1, ax2, ax3) = plt.subplots(3,figsize=(8, 6))
    # fig.suptitle('Robot position(m) vs time(s)')
    # ax1.plot(path[:,0], label = 'x', color = "#22559c")
    # ax1.legend()
    # ax1.set_xlim([0,len])
    # ax1.set_ylim([0,h])
    # ax1.set_yticks([0,h])
    # ax2.set_xlim([0,len])
    # ax2.plot(path[:,1], label = 'y', color = "#f27370")
    # ax2.set_ylim([0,w])
    # ax2.set_yticks([0,w])
    # ax2.legend()
    plt.close()
    plt.figure(figsize=(6, 2))
    plt.plot(path[:,2],label = 'z', color = "#fdb44b")
    plt.plot(z_floor, label = 'seafloor terrain height', color = "#76b39d")
    # plt.set_xlim([0,len])
    # plt.set_ylim([z_min, z_max])
    # plt.set_yticks([z_min,z_max])
    plt.xlim([0,len])
    plt.ylim([z_min-1, z_max])
    plt.yticks([z_min,z_max])
    plt.legend()
    plt.savefig(file_path)
    print("Done!")


if __name__ == "__main__":

    main(sys.argv[1:])