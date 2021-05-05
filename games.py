import numpy as np
import matplotlib.pyplot as plt
from skimage import graph
import dijkstra3d
import cv2 as cv
import skimage.graph as sg


def main():
    W = 100
    H = 150
    L = 5
    l = int(L/2)
    S = 10
    s0 = (50, 10)
    t0 = (50, 90)
    s1 = (10, 50)
    t1 = (90, 50)
    # my_map = build_map('empty', W, H, S)
    my_map = build_map('obstacle', W, H, S)

    map_3d = np.zeros((W+2, H+2, 1))
    map_3d[:,:,0] = my_map
    path_3d = sg.route_through_array((map_3d>0)*10000000+1, [s1[0], s1[1], 0], [t1[0], t1[1], 0],geometric=True)

    # path_3d = dijkstra3d.dijkstra((map_3d>0).astype(float), (s1[0], s1[1], 0), (t1[0], t1[1], 0), compass=False)
    print("cost:" , path_3d[1])
    map_3d = embed_2d_path_in_2d_map(map_3d, path_3d[0], l)
    plt.imshow(map_3d[:,:,0])
    plt.show()
    a=1



def embed_2d_path_in_2d_map(map_3d, robot_path, l):
    map_3d_0_path = map_3d.copy()
    for i in range(len(robot_path)-1):
        loc = robot_path[i]
        map_3d_0_path[loc[0]-l:loc[0]+l, loc[1]-l:loc[1]+l, 0] = 4
    return map_3d_0_path

def build_map(type, W, H, S):
    my_map = np.zeros((W + 2, H + 2))
    my_map[0] = 1
    my_map[:, 0] = 1
    my_map[W + 1] = 1
    my_map[:, H + 1] = 1
    if type == 'empty':
        return my_map

    if type == 'obstacle':
        my_map[50-S:50+S, 50-S:50+S, ] = 1
        return my_map


if __name__ == '__main__':
    main()