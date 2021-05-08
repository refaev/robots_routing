import numpy as np
from skimage import graph
import skimage.graph as sg
import cv2 as cv
import mapper


class Solver:

    def solve_two_robots_routing(self, map_2d, s0, t0, s1, t1, l):
        path_2d_rob_0 = self.find_shortest_path_2d(map_2d, s0, t0, l)
        path_2d_rob_1 = self.find_shortest_path_2d(map_2d, s1, t1, l)
        is_conflict = self.find_conflict(path_2d_rob_0, path_2d_rob_1, l)

        shortest_2d_path_length = len(path_2d_rob_0)

        map_3d_depth = shortest_2d_path_length + 4 * l
        map_3d = np.repeat(map_2d[:, :, np.newaxis], map_3d_depth, axis=2)
        map_3d_0_path = mapper.embed_2d_path_in_3d_map(map_3d, path_2d_rob_0, l)
        if not is_conflict:
            map_3d_both_path = mapper.embed_2d_path_in_3d_map(map_3d_0_path, path_2d_rob_1, l, enum=5)
            return map_3d_both_path, path_2d_rob_0, path_2d_rob_1
        else:
            map_3d_0_path_fat = mapper.dilate_map_xy(map_3d_0_path, l)
            map_3d_0_path_fat = mapper.embed_2d_path_in_3d_map(map_3d_0_path_fat, path_2d_rob_0, l + l)

            robot_1_path = self.find_shortest_path_3d(map_3d_0_path_fat, s1, t1)
            map_3d_both_path = mapper.embed_path_in_3d_map(map_3d_0_path, robot_1_path, l, enum=5)

            return map_3d_both_path, path_2d_rob_0, robot_1_path

    def find_conflict(self, path_2d_rob_0, path_2d_rob_1, l):
        for i in range(np.min((len(path_2d_rob_0), len(path_2d_rob_1)))):
            loc_0 = np.asarray(path_2d_rob_0[i])
            loc_1 = np.asarray(path_2d_rob_1[i])
            diff = np.abs((loc_0 - loc_1))
            if np.max(diff) <= l + l + 1:
                return True
        return False


    def find_shortest_path_3d(self, map_3d, s, t):
        path_3d, _ = sg.route_through_array((map_3d > 0) * 10000000 + 1, [s[0], s[1], 0], [t[0], t[1], map_3d.shape[2] - 1], geometric=True)

        # import dijkstra3d
        # graph = np.zeros(map_3d.shape, dtype=np.uint32)
        # graph += 8141840  ## move only up: 00000000011111000011110000010000
        # path_3d = dijkstra3d.dijkstra(1000000*(map_3d>0)+1, (s[0], s[1], 0), (t[0], t[1], map_3d.shape[2]-1), voxel_graph=graph, compass=True)

        return path_3d

    def find_shortest_path_2d(self, map_2d, s, t, l):
        map_2d_dilated = cv.dilate(map_2d, kernel=np.ones((l, l)))
        costs = np.where(map_2d_dilated, 1000, 1)
        path, cost = graph.route_through_array(costs, start=s, end=t, fully_connected=True)
        return path
