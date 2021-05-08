import matplotlib.pyplot as plt

import simulator
from solver import Solver
import visualizer


def main():
    W = 120
    H = 150
    l = 1

    show_example('empty', W, H, l)
    show_example('obstacle', W, H, l)
    show_example('obstacle_conflict', W, H, l)
    show_example('corridor', W, H, l)


def show_example(case, W, H, l):
    my_solver = Solver()
    map_2d, s0, t0, s1, t1, S = simulator.build_map(case, W, H)
    map_3d, robot_0_path, robot_1_path = my_solver.solve_two_robots_routing(map_2d, s0, t0, s1, t1, l)
    visualizer.show_3d_as_video(map_3d, s0, t0, s1, t1, case)

if __name__ == '__main__':
    main()
    plt.show()