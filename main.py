import numpy as np
import matplotlib.pyplot as plt
from skimage import graph
import dijkstra3d
import cv2 as cv


def main():
    W = 120
    H = 150
    L = 2
    l = int(L/2)
    # S = 10

    map_2d, s0, t0, s1, t1, S = build_map('corridor', W, H)
    # map_2d, s0, t0, s1, t1, S = build_map('empty', W, H, S)
    # map_2d, s0, t0, s1, t1, S = build_map('obstacle', W, H, S)

    solve_two_robots_routing(map_2d, s0, t0, s1, t1, l)


def solve_two_robots_routing(map_2d, s0, t0, s1, t1, l):
    path_2d = find_shortest_path_2d(map_2d, s0, t0, l)
    shortest_2d_path_length = len(path_2d)

    map_3d_depth = shortest_2d_path_length + 4*l
    map_3d = np.repeat(map_2d[:, :, np.newaxis], map_3d_depth, axis=2)
    map_3d_0_path = embed_2d_path_in_3d_map(map_3d, path_2d, l)
    map_3d_0_path_fat = dilate_map_xy(map_3d_0_path, l)
    map_3d_0_path_fat = embed_2d_path_in_3d_map(map_3d_0_path_fat, path_2d, l+l)

    robot_1_path = find_shortest_path_3d(map_3d_0_path_fat, s1, t1)
    map_3d_0_1_path = embed_path_in_3d_map(map_3d_0_path, robot_1_path, l, 5)
    show_3d_as_video(map_3d_0_1_path)

    plt.show()
    a=1


def dilate_map_xy(map_3d, l):
    map_3d_copy = map_3d.copy()
    kernel = np.ones((l+l+1, l+l+1))
    for i in range(map_3d_copy.shape[2]):
        im = map_3d_copy[:,:,i]
        im_fat = cv.dilate((im>0).astype(float), kernel=kernel)
        map_3d_copy[:,:,i] = im_fat
    return map_3d_copy


def show_3d_as_video(map_3d_0_path):
    for i in range(map_3d_0_path.shape[2]):
        im = map_3d_0_path[:,:,i]
        plt.clf()
        plt.imshow(im)
        plt.title(str(i))
        plt.pause(0.1)


def embed_2d_path_in_2d_map(map_3d, robot_path, l):
    map_3d_0_path = map_3d.copy()
    for i in range(len(robot_path)-1):
        loc = robot_path[i]
        map_3d_0_path[loc[0]-l:loc[0]+l, loc[1]-l:loc[1]+l] = 4
    return map_3d_0_path


def embed_2d_path_in_3d_map(map_3d, robot_path, l):
    map_3d_0_path = map_3d.copy()
    for i in range(len(robot_path)-1):
        loc = robot_path[i]
        map_3d_0_path[loc[0]-l+1:loc[0]+l+1, loc[1]-l+1:loc[1]+l+1, i] = 4
    for j in range(i, map_3d.shape[2]):
        loc = robot_path[i]
        map_3d_0_path[loc[0]-l+1:loc[0]+l+1, loc[1]-l+1:loc[1]+l+1, j] = 4
    return map_3d_0_path


def embed_path_in_3d_map(map_3d, robot_0_path, l, enum=4):
    map_3d_0_path = map_3d.copy()
    for i in range(len(robot_0_path)):
        loc = robot_0_path[i]
        map_3d_0_path[loc[0]-l:loc[0]+l, loc[1]-l:loc[1]+l, loc[2]] = enum
    return map_3d_0_path


def find_shortest_path_3d(map_3d, s, t):
    graph = np.zeros(map_3d.shape, dtype=np.uint32)
    graph += 8141840 ## move only up: 00000000011111000011110000010000
    # graph += 15376 ## move only up, 4 neighbours or stay: 00000000000000000011110000010000
    # path_3d = dijkstra3d.dijkstra(1000000*(map_3d>0)+1, (s[0], s[1], 0), (t[0], t[1], map_3d.shape[2]-1), connectivity=26, compass=True)
    path_3d = dijkstra3d.dijkstra(1000000*(map_3d>0)+1, (s[0], s[1], 0), (t[0], t[1], map_3d.shape[2]-1), voxel_graph=graph, compass=True)
    return path_3d


def find_shortest_path_2d(map_2d, s, t, l):
    map_2d_dilated = cv.dilate(map_2d, kernel=np.ones((l, l)))
    costs = np.where(map_2d_dilated, 1000, 1)
    path, cost = graph.route_through_array(costs, start=s, end=t, fully_connected=True)
    # plt.imshow(map_2d)
    # plt.plot(np.asarray(path)[:,0],np.asarray(path)[:,1],'og')
    # plt.show()
    return path


def render_map(map_2d, s0, t0, s1, t1, l):
    im = map_2d.copy()
    im[t0[0], t0[1]] = 2
    im[t1[0], t1[1]] = 3
    im[s0[0]-l:s0[0]+l, s0[1]-l:s0[1]+l] = 2
    im[s1[0]-l:s1[0]+l, s1[1]-l:s1[1]+l] = 3
    return im


def build_map(type, W, H):
    map_2d = np.zeros((W + 2, H + 2))
    map_2d[0] = 1
    map_2d[:, 0] = 1
    map_2d[W + 1] = 1
    map_2d[:, H + 1] = 1
    if type == 'empty':
        s0 = (50, 10)
        t0 = (50, 90)
        s1 = (10, 50)
        t1 = (90, 50)
        S = 10
        return map_2d, s0, t0, s1, t1, S

    if type == 'obstacle':
        S = 10
        map_2d[50-S:50+S, 50-S:50+S, ] = 1
        s0 = (50, 10)
        t0 = (50, 90)
        s1 = (10, 50)
        t1 = (90, 50)
        return map_2d, s0, t0, s1, t1, S

    if type == 'corridor':
        S = 18
        map_2d[30-S:30+S, 20-S:20+S, ] = 1
        map_2d[70-S:70+S, 60-S:60+S, ] = 1
        map_2d[30-S:30+S, 60-S:60+S, ] = 1
        map_2d[70-S:70+S, 20-S:20+S, ] = 1

        map_2d[30-S:30+S, 132-S:132+S, ] = 1
        map_2d[70-S:70+S, 96-S:96+S, ] = 1
        map_2d[30-S:30+S, 96-S:96+S, ] = 1
        map_2d[70-S:70+S, 132-S:132+S, ] = 1

        s0 = (6, 30)
        t0 = (97, 30)
        s1 = (6, 50)
        t1 = (97, 50)
        return map_2d, s0, t0, s1, t1, S


if __name__ == '__main__':
    main()
    plt.show()