import numpy as np
import cv2 as cv


def dilate_map_xy(map_3d, l):
    map_3d_copy = map_3d.copy()
    kernel = np.ones((l+l+1, l+l+1))
    for i in range(map_3d_copy.shape[2]):
        im = map_3d_copy[:,:,i]
        im_fat = cv.dilate((im>0).astype(float), kernel=kernel)
        map_3d_copy[:,:,i] = im_fat
    return map_3d_copy


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

