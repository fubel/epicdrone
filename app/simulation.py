from __future__ import division

from psdrone.simulation import *
from world import World
from direct.gui.OnscreenText import OnscreenText, TextNode

if __name__ == '__main__':
    # initialize Drone
    D = DummyDrone(np.array([3.5, 1., 2.]), 0)
    K = Kalman1D(D)

    movements = [np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]),
                 np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]),
                 np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]),
                 np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0])]

    # mesh for kalman gaussian vis
    x = np.linspace(D.x_range[0], D.x_range[1], 500)

    world = World()

    # one marker in the middle of the four walls
    world.set_default_markers()

    # create custom line opbjects for later use
    def initialize_world(world):
        world.l1 = world.line_model(0., 1., 0.)
        world.l2 = world.line_model(1., 0., 0.)
        world.l3 = world.line_model(0., 0., 1.)
        world.l4 = world.line_model(0., 1., 1.)
        world.l5 = world.line_model(1., 1., 0.)

    world.hook_init(initialize_world)

    def sim_loop(world, task):
        if task.frame % 60 == 0:
            for movement in movements + movements[::-1]:

                K.predict(movement)
                m = D.measure()
                K.update(m)
                D.update(m)

                drone_position2 = world.convert_position(D.real_position)
                world.drone.setPos(drone_position2[0], drone_position2[1], drone_position2[2])
                world.drone.setHpr(-world.drone_position[3], world.drone_position[4], world.drone_position[5])

                dir = np.array([0, 0, 1], dtype=np.float32)
                dir = 1 / (2 * np.linalg.norm(dir)) * dir

                world.line_draw(world.l1,
                               [0, 0, 0],
                               world.convert_position(D.real_position))
                world.line_draw(world.l2,
                               world.convert_position(D.real_position),
                               world.convert_position([D.real_position[0] + dir[0], D.real_position[1] + dir[1],
                                                      D.real_position[2] + dir[2]]))
                world.line_draw(world.l3,
                               [0, 0, 0],
                               world.convert_position(D.estimated_position))
                world.line_draw(world.l4,
                               world.convert_position(D.estimated_position),
                               world.convert_position(
                                   [D.estimated_position[0] + dir[0], D.estimated_position[1] + dir[1],
                                    D.estimated_position[2] + dir[2]]))

                #for key, value in D.landmarks.iteritems():
                #    world.line_draw(world.marker_lines[key], [0, 0, 0], world.convert_position(D.landmarks[key]))
                #    if key in D.observable_landmarks():
                #        world.line_draw(world.marker_lines_observed[key], world.convert_position(D.landmarks[key]),
                #                       world.convert_position(D.real_position))
                #    else:
                #        world.marker_lines_observed[key].reset()

                if world.fps_text4 is not None:
                    world.fps_text4.destroy()
                world.fps_text4 = OnscreenText(
                    text='$MSE_A = $%s, $MSE_K = $%s' % (D.error, (D.real_position[0] - K.state[0]) ** 2),
                    pos=(0.05, 0.20),
                    scale=0.05,
                    align=TextNode.ALeft,
                    parent=world.a2dBottomLeft)
            # for movement in movements + movements[::-1]:
            #     K.predict(movement)
            #     m = D.measure()
            #     K.update(m)
            #     D.update(m)
            #
            #
            #     ax.scatter(D.real_position[0], D.real_position[1], D.real_position[2], zdir='y', c='red', label='real position')
            #     ax.scatter(D.estimated_position[0], D.estimated_position[1],
            #                D.estimated_position[2], zdir='y', c='gray', label='Average estimation')
            #     ax.scatter(K.state[0], D.real_position[1],
            #                D.real_position[2], zdir='y', c='green', label='Kalman estimation')
            #     ax.scatter(0, 0, 0, zdir='y')
            #     rv = norm(loc=K.state[0], scale=K.state[1])
            #     ax.plot(x, rv.pdf(x), zdir='y', label='Kalman Filter belief')
            #     dir = np.array([0, 0, 1], dtype=np.float32)
            #     dir = 1 / (2*np.linalg.norm(dir)) * dir
            #     arw = Arrow3D.arrow(np.array([0, 0, 0]), D.real_position, color='red', lw=3)
            #     ax.add_artist(arw)
            #     arw = Arrow3D.arrow(D.real_position, D.real_position + dir, color='red')
            #     ax.add_artist(arw)
            #     arw = Arrow3D.arrow(np.array([0, 0, 0]), D.estimated_position, color='gray', lw=3)
            #     ax.add_artist(arw)
            #     arw = Arrow3D.arrow(D.estimated_position, D.estimated_position + dir, color='gray')
            #     ax.add_artist(arw)
            #     for key, value in D.landmarks.iteritems():
            #         ax.scatter(D.landmarks[key][0], D.landmarks[key][1], D.landmarks[key][2], zdir='y', c='blue')
            #         #arw2 = Arrow3D.arrow(np.array([0, 0, 0]), D.landmarks[key], color='gray')
            #         #ax.add_artist(arw2)
            #         if key in D.observable_landmarks():
            #             arw3 = Arrow3D.arrow(D.landmarks[key], D.real_position)
            #             ax.add_artist(arw3)
            #     handles, labels = ax.get_legend_handles_labels()
            #     ax.legend(handles, labels)
            #     textstr = '$MSE_A = $%s, $MSE_K = $%s' % (D.error, (D.real_position[0] - K.state[0])**2)
            #     # these are matplotlib.patch.Patch properties
            #     props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
            #     # place a text box in upper left in axes coords
            #     ax.text(5, 12, 0, textstr, transform=ax.transAxes, fontsize=14,
            #             verticalalignment='top', bbox=props)
            #     fig.canvas.draw()
            #     time.sleep(1)
            #     ax.clear()

    world.hook_loop(sim_loop)
    world.run()
