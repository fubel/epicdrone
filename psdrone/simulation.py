import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
import random

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)


class DummyDrone(object):
    def __init__(self, x, y, z, theta, cone_height=2, cone_radius=2):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.landmarks = {
            1: np.array([3, 1, 4], dtype=np.float32),
            2: np.array([4, 1, 4], dtype=np.float32),
        }
        self.cone = (cone_height, cone_radius)
        print("Initialized DummyDrone with position %s") % self.get_real_position()

    def move(self, dx, dy, dz):
        self.x += dx
        self.y += dy
        self.z += dz
        print("Moved to position %s") % self.get_real_position()

    def turn(self, dtheta):
        self.theta += dtheta

    def get_real_position(self):
        return np.array([self.x, self.y, self.z])

    def observable_landmarks(self):
        observable = []
        dir = np.array([0, 0, 1], dtype=np.float32)
        dir = 1/np.linalg.norm(dir) * dir
        for key, value in self.landmarks.iteritems():
            cone_dist = np.dot(self.landmarks[key] - self.get_real_position(), dir)
            print cone_dist
            if cone_dist >= 0 or cone_dist <= self.cone[1]:
                cone_radius = (1.0*cone_dist / self.cone[0]) * self.cone[1]
                orth_distance = np.linalg.norm(self.landmarks[key] - self.get_real_position() - cone_dist * dir)
                print orth_distance
                print cone_radius
                if orth_distance < cone_radius:
                    observable.append(self.landmarks[key])
        return observable

    @property
    def position(self):
        return self.get_real_position()

    def get_approximate_positions(self):
        measurements = {}
        positions = {}
        for key, value in self.landmarks.iteritems():
            measurements[key] = value - self.get_real_position()
            print("Measurement to landmark %s: %s") % (key, measurements[key])
            positions[key] = value - measurements[key]
            print("Approximate position using only measurement to landmark %s : %s") % (key, value - measurements[key])
        return positions.values()

    def arrow(self, point1, point2, color='black'):
        return Arrow3D([point1[0], point2[0]],
                       [point1[2], point2[2]],
                       [point1[1], point2[1]],
                       arrowstyle="->", color=color, lw = 3, mutation_scale=25)



# initialize Drone
D = DummyDrone(3, 1, 3, 0)
print D.observable_landmarks()



#
# plt.ion()
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
#
# while True:
#     ax.set_xlim(0, 7)
#     ax.set_ylim(0, 4)
#     ax.set_zlim(0, 3)
#
#     random_movements = (random.randint(-1, 1), 0, 0)
#     if D.get_real_position()[0] + random_movements[0] <= 7:
#         if D.get_real_position()[0] + random_movements[0] >= 0:
#             D.move(random_movements[0], random_movements[1], random_movements[2])
#
#     x, y, z = D.get_real_position()
#     ax.scatter(x, y, z, zdir='y', c='red')
#     ax.scatter(0, 0, 0)
#     ax.scatter(D.landmarks[1][0], D.landmarks[1][1], D.landmarks[1][2], zdir='y', c='blue')
#     arw = D.arrow(np.array([0,0,0]), D.get_real_position(), color='gray')
#     arw2 = D.arrow(np.array([0,0,0]), D.landmarks[1], color='gray')
#     arw3 = D.arrow(D.landmarks[1], D.get_real_position())
#     ax.add_artist(arw)
#     ax.add_artist(arw2)
#     ax.add_artist(arw3)
#     fig.canvas.draw()
#     ax.clear()
