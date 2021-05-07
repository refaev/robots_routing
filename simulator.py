import numpy as np


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

