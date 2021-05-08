import matplotlib.pyplot as plt


def show_3d_as_video(map_3d, s0, t0, s1, t1, case):
    for i in range(map_3d.shape[2]):
        im = map_3d[:, :, i]
        plt.clf()
        plt.imshow(im)
        plt.plot(s0[1], s0[0], 'og')
        plt.plot(t0[1], t0[0], 'xg')
        plt.plot(s1[1], s1[0], 'oy')
        plt.plot(t1[1], t1[0], 'xy')
        plt.title(case + " " + str(i))
        plt.pause(0.01)
