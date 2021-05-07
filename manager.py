import matplotlib.pyplot as plt

import simulator
from solver import Solver
import visualizer


def main():
    W = 120
    H = 150
    l = 1

    my_solver = Solver()

    map_2d, s0, t0, s1, t1, S = simulator.build_map('empty', W, H)
    map_3d = my_solver.solve_two_robots_routing(map_2d, s0, t0, s1, t1, l)
    visualizer.show_3d_as_video(map_3d, s0, t0, s1, t1)

    map_2d, s0, t0, s1, t1, S = simulator.build_map('obstacle', W, H)
    map_3d = my_solver.solve_two_robots_routing(map_2d, s0, t0, s1, t1, l)
    visualizer.show_3d_as_video(map_3d, s0, t0, s1, t1)

    map_2d, s0, t0, s1, t1, S = simulator.build_map('corridor', W, H)
    map_3d = my_solver.solve_two_robots_routing(map_2d, s0, t0, s1, t1, l)
    visualizer.show_3d_as_video(map_3d, s0, t0, s1, t1)


if __name__ == '__main__':
    main()
    plt.show()