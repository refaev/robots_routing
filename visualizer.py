import matplotlib.pyplot as plt


def show_3d_as_video(map_3d):
    for i in range(map_3d.shape[2]):
        im = map_3d[:, :, i]
        plt.clf()
        plt.imshow(im)
        plt.title(str(i))
        plt.pause(0.01)
