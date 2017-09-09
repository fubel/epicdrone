import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch



class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

    @staticmethod
    def arrow(point1, point2, color='black', lw=1):
        return Arrow3D([point1[0], point2[0]],
                       [point1[2], point2[2]],
                       [point1[1], point2[1]],
                       arrowstyle="->", color=color, lw=lw, mutation_scale=25)


class DummyDrone(object):
    def __init__(self, position, theta, cone_height=3, cone_radius=3):
        # drone pose
        self.real_position = position
        self.estimated_position = position
        self.orientation = theta

        # landmarks
        self.landmarks = {
            0: np.array([2, 1, 4], dtype=np.float32),
            1: np.array([3, 1, 4], dtype=np.float32),
            2: np.array([4, 1, 4], dtype=np.float32),
            3: np.array([5, 1, 4], dtype=np.float32),
        }

        # vision
        self.cone = (cone_height, cone_radius)

        # room
        self.x_range = (0, 7)
        self.y_range = (0, 3)
        self.z_range = (0, 4)

        # noises
        self.measurement_cov = np.array([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]])
        self.movement_cov = np.array([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]])
        self.process_cov = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])

        print("Initialized DummyDrone with position %s") % self.real_position

    @property
    def error(self):
        return np.abs(self.estimated_position - self.real_position)

    def move(self, movement):
        """
        Takes a "perfect" movement and performs a noisy move
        Args:
            movement (np.array): Perfect movement
        """
        print("Trying to make movement: %s" % movement)
        # draw from noisy distribution centered around movement coordinates to compute real movement
        noisy = np.random.multivariate_normal(movement, self.movement_cov)
        # update the real position by the real movement
        self.real_position += noisy
        print("Current real position: %s") % self.real_position

    def observable_landmarks(self):
        """
        Given the drones eye cone, decide which landmarks are observable from current real position
        Returns:
            A list of landmark keys
        """
        observable = []
        dir = np.array([0, 0, 1], dtype=np.float32)
        dir = 1 / np.linalg.norm(dir) * dir
        for key, value in self.landmarks.iteritems():
            cone_dist = np.dot(self.landmarks[key] - self.real_position, dir)
            if cone_dist >= 0 or cone_dist <= self.cone[1]:
                cone_radius = (1.0 * cone_dist / self.cone[0]) * self.cone[1]
                orth_distance = np.linalg.norm(self.landmarks[key] - self.real_position - cone_dist * dir)
                if orth_distance < cone_radius:
                    observable.append(key)
        return observable

    def measure(self):
        """
        Goes through the list of observable landmarks, measures "perfect" distance and distorts it.
        Returns:
            (Fused) measurement
        """
        measurements = {}
        observable = self.observable_landmarks()
        for key in observable:
            mean = self.real_position - self.landmarks[key]
            print(mean)
            measurements[key] = self.landmarks[key] + np.random.multivariate_normal(mean, self.measurement_cov)
        # do the simplest possible fusion by averaging:
        if observable:
            m = 1.0/len(measurements) * sum(measurements.values())
            print("Measured (fused): %s" % m)
            self.estimated_position = 0.5*(self.estimated_position + m)
        print("Current estimated position: %s" % self.estimated_position)
        print("Current error: %s" % self.error)

# initialize Drone
D = DummyDrone(np.array([3.5, 1., 2.]), 0)

movements = [np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]),
             np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]),
             np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0])]

plt.ion()
fig = plt.figure(figsize=(14,12))
ax = fig.add_subplot(111, projection='3d')

for movement in movements:
    ax.set_xlim(0, 7)
    ax.set_ylim(0, 4)
    ax.set_zlim(0, 3)
    D.move(movement)
    D.measure()
    ax.scatter(D.real_position[0], D.real_position[1], D.real_position[2], zdir='y', c='red', label='real position')
    ax.scatter(D.estimated_position[0], D.estimated_position[1],
               D.estimated_position[2], zdir='y', c='gray', label='estimated position')
    ax.scatter(0, 0, 0, zdir='y')
    dir = np.array([0, 0, 1], dtype=np.float32)
    dir = 1 / np.linalg.norm(dir) * dir
    arw = Arrow3D.arrow(np.array([0, 0, 0]), D.real_position, color='red', lw=3)
    ax.add_artist(arw)
    arw = Arrow3D.arrow(D.real_position, D.real_position + dir, color='red')
    ax.add_artist(arw)
    arw = Arrow3D.arrow(np.array([0, 0, 0]), D.estimated_position, color='gray', lw=3)
    ax.add_artist(arw)
    arw = Arrow3D.arrow(D.estimated_position, D.estimated_position + dir, color='gray')
    ax.add_artist(arw)
    for key, value in D.landmarks.iteritems():
        ax.scatter(D.landmarks[key][0], D.landmarks[key][1], D.landmarks[key][2], zdir='y', c='blue')
        arw2 = Arrow3D.arrow(np.array([0, 0, 0]), D.landmarks[key], color='gray')
        ax.add_artist(arw2)
        if key in D.observable_landmarks():
            arw3 = Arrow3D.arrow(D.landmarks[key], D.real_position)
            ax.add_artist(arw3)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)
    fig.canvas.draw()
    time.sleep(2)
    ax.clear()