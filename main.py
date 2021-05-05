import numpy as np
import matplotlib.pyplot as plt
from skimage import graph
import dijkstra3d
import cv2 as cv


def main():
    W = 120
    H = 150
    L = 5
    l = int(L/2)
    # S = 10

    my_map, s0, t0, s1, t1, S = build_map('corridor', W, H)
    # my_map = build_map('empty', W, H, S)
    # my_map = build_map('obstacle', W, H, S)

    solve_two_robots_routing(my_map, s0, t0, s1, t1, S, l)

def solve_two_robots_routing(my_map, s0, t0, s1, t1, S, l):
    path_2d = find_shortest_path_2d(my_map, s0, t0, l)
    shortest_2d_path_length = len(path_2d)

    map_3d_depth = shortest_2d_path_length + 4*l
    map_3d = np.repeat(my_map[:, :, np.newaxis], map_3d_depth, axis=2)
    map_3d_0_path = embed_2d_path_in_3d_map(map_3d, path_2d, l)
    map_3d_0_path_fat = embed_2d_path_in_3d_map(map_3d, path_2d, l+l+1)

    robot_1_path = find_shortest_path_3d(map_3d_0_path_fat, s1, t1)
    map_3d_0_1_path = embed_path_in_3d_map(map_3d_0_path, robot_1_path, l, 5)
    show_3d_as_video(map_3d_0_1_path)

    a=1


def show_3d_as_video(map_3d_0_path):
    for i in range(map_3d_0_path.shape[2]):
        im = map_3d_0_path[:,:,i]
        plt.clf()
        plt.imshow(im)
        plt.title(str(i))
        plt.pause(0.1)


def embed_2d_path_in_3d_map(map_3d, robot_path, l):
    map_3d_0_path = map_3d.copy()
    for i in range(len(robot_path)-1):
        loc = robot_path[i]
        map_3d_0_path[loc[0]-l:loc[0]+l, loc[1]-l:loc[1]+l, i] = 4
    for j in range(i, map_3d.shape[2]):
        loc = robot_path[i]
        map_3d_0_path[loc[0]-l:loc[0]+l, loc[1]-l:loc[1]+l, j] = 4
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
    # map_3d = (map_3d>0)*100 + 1
    # graph[map_3d > 0] = 0
    path_3d = dijkstra3d.dijkstra(1000000*(map_3d>0)+1, (s[0], s[1], 0), (t[0], t[1], map_3d.shape[2]-1), voxel_graph=graph)#, compass=True)
    return path_3d


def find_shortest_path_2d(my_map, s, t, l):
    my_map_dilated = cv.dilate(my_map, kernel=np.ones((l+l+1, l+l+1)))
    costs = np.where(my_map_dilated, 1000, 1)
    path, cost = graph.route_through_array(costs, start=s, end=t, fully_connected=True)
    # plt.imshow(my_map)
    # plt.plot(np.asarray(path)[:,0],np.asarray(path)[:,1],'og')
    # plt.show()
    return path


def render_map(my_map, s0, t0, s1, t1, l):
    im = my_map.copy()
    im[t0[0], t0[1]] = 2
    im[t1[0], t1[1]] = 3
    im[s0[0]-l:s0[0]+l+1, s0[1]-l:s0[1]+l+1] = 2
    im[s1[0]-l:s1[0]+l+1, s1[1]-l:s1[1]+l+1] = 3
    return im


def build_map(type, W, H):
    my_map = np.zeros((W + 2, H + 2))
    my_map[0] = 1
    my_map[:, 0] = 1
    my_map[W + 1] = 1
    my_map[:, H + 1] = 1
    if type == 'empty':
        s0 = (50, 10)
        t0 = (50, 90)
        s1 = (10, 50)
        t1 = (90, 50)
        S = 10
        return my_map, s0, t0, s1, t1, S

    if type == 'obstacle':
        S = 10
        my_map[50-S:50+S, 50-S:50+S, ] = 1
        s0 = (50, 10)
        t0 = (50, 90)
        s1 = (10, 50)
        t1 = (90, 50)
        return my_map, s0, t0, s1, t1, S

    if type == 'corridor':
        S = 18
        my_map[30-S:30+S, 30-S:30+S, ] = 1
        my_map[70-S:70+S, 70-S:70+S, ] = 1
        my_map[30-S:30+S, 70-S:70+S, ] = 1
        my_map[70-S:70+S, 30-S:30+S, ] = 1
        s0 = (6, 6)
        t0 = (94, 94)
        s1 = (6, 94)
        t1 = (94, 6)
        return my_map, s0, t0, s1, t1, S
if __name__ == '__main__':
    main()