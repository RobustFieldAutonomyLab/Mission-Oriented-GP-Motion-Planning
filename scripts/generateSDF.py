import sys, getopt
import numpy as np
import pandas as pd
import math
from scipy import ndimage
import matplotlib.pyplot as plt


def main(argv):
    input_file = ''
    output_file = ''
    cell_size = 1
    cell_size_z = 1
    floor_level = 0

    options = "h:i:o:c:z:f:l:"
    long_options = ["Help", "Input", "Output", "Cell_size", "Z_cell_size", "Seafloor_bottom_level", "Sea_surface_level"]

    opts, args = getopt.getopt(argv, options, long_options)
    for opt, arg in opts:
        if opt in ("-h", "--Help"):
            print('test.py -i <input Seafloor file> -o <output SDF file> -c <cell size> -z <z cell size> -f <see floor bottom>')
            sys.exit()
        elif opt in ("-i", "--Input"):
            input_file = arg
        elif opt in ("-o", "--Output"):
            output_file = arg
        elif opt in ("-c", "--Cell_size"):
            cell_size = float(arg)
        elif opt in ("-z", "--Z_cell_size"):
            cell_size_z = float(arg)
        elif opt in ("-f", "--Seafloor_bottom_level"):
            floor_level = float(arg)
        elif opt in ("-l", "--Sea_surface_level"):
            surface_level = float(arg)


    seafloor_map = readSeafloorMap(input_file)
    field = grid23Dfield(seafloor_map, cell_size_z, floor_level, surface_level)
    sdf_data = signedDistanceField3D(field, cell_size)
    saveSDF(sdf_data, output_file, cell_size, cell_size_z, floor_level, surface_level)

def readSeafloorMap(seafloor_path):
    file = open(seafloor_path, "r")
    data = file.read()
    data2list = data.replace('\n', ' ').split(" ")
    data2list.remove("")
    size_array = np.array(data2list[:2], dtype=np.float64)
    x_width = int(size_array[0])
    y_width = int(size_array[1])
    seafloor_map  = np.array(data2list[2:]).astype(float)
    seafloor_map = seafloor_map.reshape((x_width, y_width))
    plt.imshow(seafloor_map, origin='lower')
    plt.savefig("maploaded.png")
    return seafloor_map

def grid23Dfield(grid, cell_size_z, floor_level, surface_level):
    [rows, cols] = grid.shape
    g_max = np.max(grid)
    g_min = np.min(grid)

    z_max = min(g_max, surface_level)
    z_min = min(g_min, floor_level)

    z_level = int((z_max - z_min) / float(cell_size_z) + 0.5)
    field = np.zeros((z_level, rows, cols))
    for z in range(0, z_level):
        for i in range(0, rows):
            for j in range(0, cols):
                if grid[i,j]>=0:
                    field[z,i,j] = 1
                elif z * cell_size_z + floor_level < grid[i,j]:
                    field[z,i,j] = 1
    return field

# from GPMP2 python wrap
def signedDistanceField3D(ground_truth_map, cell_size):
    # SIGNEDDISTANCEFIELD3D 3D signed distance field
    #   Given a ground truth 3D map defined in Matrix in 0-1,
    #   calculate 3D signed distance field, which is defined as a matrix
    #   map matrix and signed distance field matrix have the same resolution.
    #
    #   Usage: field = SIGNEDDISTANCEFIELD3D(ground_truth_map, cell_siz)
    #   @map        evidence grid from dataset, map use 0 show open area, 1 show objects.
    #   @cell_size  cell sizeto given metric information
    #
    #   Output:
    #   @field      sdf, row is X, col is Y, 3rd index is Z

    # regularize unknow area to open area
    cur_map = ground_truth_map > 0.75
    cur_map = cur_map.astype(int)

    if np.amax(cur_map) == 0:
        return np.ones(ground_truth_map.shape) * 1000

    # inverse map
    inv_map = 1 - cur_map

    # get signed distance from map and inverse map
    # since bwdist(foo) = ndimage.distance_transform_edt(1-foo)
    map_dist = ndimage.distance_transform_edt(inv_map)
    inv_map_dist = ndimage.distance_transform_edt(cur_map)

    field = map_dist - inv_map_dist

    # metric
    field = field * cell_size
    field = field.astype(float)

    return field

def saveSDF(field, path, cell_size, cell_size_z, floor_level, surface_level):
    [z, x, y] = field.shape
    data = field.flatten()
    data = np.insert(data, 0, [z,x,y,cell_size, cell_size_z, floor_level, surface_level], axis=0)
    np.savetxt(path, data)
    print("Success!")


if __name__ == "__main__":

    main(sys.argv[1:])
