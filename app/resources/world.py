from __future__ import division

from panda3d.core import loadPrcFile

loadPrcFile("config/Config.prc")

import numpy as np
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import TransparencyAttrib
from direct.gui.OnscreenText import OnscreenText, TextNode
from direct.directtools.DirectGeometry import LineNodePath
from drone import Drone


class World(ShowBase):

    fps_text = fps_text2 = fps_text3 = fps_text4 = None
    room_dimentions = []
    camera_position = [1468, 1177, 1160, -126, -38, 0]  # x y z h p r
    drone = None
    drone_instance = None
    markers = {}
    marker_lines = {}
    marker_lines_observed = {}
    active_keys = {}
    loop_callback = None

    def __init__(self, width = 6.78, length = 5.82, simulation=True):
        ShowBase.__init__(self)

        width *= 100
        length *= 100
        self.room_dimentions = [width, length]

        self.wall1 = self.wall_model(0, 0, 0, width, 0)
        self.wall2 = self.wall_model(0, 0, 0, length, 90)
        self.wall3 = self.wall_model(width, length, 0, width, 180)
        self.wall4 = self.wall_model(width, length, 0, length, -90)

        self.drone = self.drone_model()
        self.drone_instance = Drone(simulation=simulation)

        # Add the spinCameraTask procedure to the task manager.
        self.tick_loop = self.taskMgr.add(self.tick, "tick_loop")

        self.keypress_repeat("w", self.move_drone, ["x", 1])
        self.keypress_repeat("s", self.move_drone, ["x", -1])
        self.keypress_repeat("a", self.move_drone, ["y", 1])
        self.keypress_repeat("d", self.move_drone, ["y", -1])
        self.keypress_repeat("r", self.move_drone, ["z", 1])
        self.keypress_repeat("f", self.move_drone, ["z", -1])
        self.keypress_repeat("4", self.move_camera, ["x", -1])
        self.keypress_repeat("6", self.move_camera, ["x", 1])
        self.keypress_repeat("8", self.move_camera, ["y", 1])
        self.keypress_repeat("5", self.move_camera, ["y", -1])
        self.keypress_repeat("1", self.move_camera, ["z", 1])
        self.keypress_repeat("3", self.move_camera, ["z", -1])
        self.keypress_repeat("7", self.move_camera, ["h", -1])
        self.keypress_repeat("9", self.move_camera, ["h", 1])
        self.keypress_repeat("arrow_up", self.move_camera, ["p", 1])
        self.keypress_repeat("arrow_down", self.move_camera, ["p", -1])

    def keypress_repeat(self, key, callback, parameter):
        self.accept(key, self.keypress_start, [key, callback, parameter])
        self.accept(key + "-up", self.keypress_stop, [key])

    def keypress_start(self, key, callback, parameter):
        self.active_keys[key] = [callback, parameter]

    def keypress_stop(self, key):
        self.active_keys[key] = None

    def move_camera(self, parameter):
        if parameter[0] == "x":
            self.camera_position[0] += 10 * np.cos(np.deg2rad(self.camera_position[3])) * parameter[1]
            self.camera_position[1] += -10 * np.sin(np.deg2rad(self.camera_position[3])) * parameter[1]
        if parameter[0] == "y":
            self.camera_position[0] += 10 * np.sin(np.deg2rad(self.camera_position[3])) * parameter[1]
            self.camera_position[1] += 10 * np.cos(np.deg2rad(self.camera_position[3])) * parameter[1]
        if parameter[0] == "z":
            self.camera_position[2] += 10 * parameter[1]
        if parameter[0] == "h":
            self.camera_position[3] += parameter[1]
        if parameter[0] == "p":
            self.camera_position[4] += parameter[1]

    def move_drone(self, axis, direction):
        pass
        '''
        if axis == "x":
            self.drone_position[0] += 5 * direction
        if axis == "y":
            self.drone_position[1] += 5 * direction
        if axis == "z":
            self.drone_position[2] += 1 * direction
        if axis == "h":
            self.drone_position[3] += direction
        '''

    def tick(self, task):

        for key in self.active_keys:
            if self.active_keys[key] is not None:
                self.active_keys[key][0](self.active_keys[key][1])

        if self.loop_callback is not None:
            self.loop_callback(self, task)

        self.camera.setPos(self.camera_position[0], self.camera_position[1], self.camera_position[2])
        self.camera.setHpr(-self.camera_position[3], self.camera_position[4], self.camera_position[5])

        drone_position = self.convert_position(self.drone_instance.get_position())
        drone_orientation = self.drone_instance.get_orientation()
        self.drone.setPos(drone_position[0], drone_position[1], drone_position[2])
        self.drone.setHpr(drone_orientation[0], drone_orientation[1], drone_orientation[2])

        if task.time > 0:
            if self.fps_text is not None:
                self.fps_text.destroy()
            self.fps_text = OnscreenText(text="Tick-Rate: " + str(int(task.frame / task.time)),
                                         pos=(0.05, 0.05),
                                         scale=0.05,
                                         align=TextNode.ALeft,
                                         parent=self.a2dBottomLeft)

            if self.fps_text2 is not None:
                self.fps_text2.destroy()
            self.fps_text2 = OnscreenText(text="Camera Position: " + str(self.camera_position),
                                          pos=(0.05, 0.1),
                                          scale=0.05,
                                          align=TextNode.ALeft,
                                          parent=self.a2dBottomLeft)

        return Task.cont

    def drone_model(self):
        drone = self.loader.loadModel("resources/models_3d/drone")
        drone.setScale(15, 15, 10)
        drone.reparentTo(self.render)
        return drone

    def wall_model(self, x, y , z, length, orientation):
        wall = self.loader.loadModel("resources/models_3d/plane")
        wall.reparentTo(self.render)
        wall.setScale(length, 0, 290)
        wall.setPos(x, y, z)
        wall.setH(orientation)
        wall.setTransparency(TransparencyAttrib.MAlpha)
        wall.setAlphaScale(0.1)
        return wall

    def marker_model(self, position, orientation):
        marker = self.loader.loadModel("resources/models_3d/plane")
        marker.reparentTo(self.render)
        marker.setScale(21, 0, 21)
        marker.setPos(position[0], position[1], position[2])
        marker.setHpr(orientation[0], orientation[1], orientation[2])
        marker.setTransparency(TransparencyAttrib.MAlpha)
        marker.setAlphaScale(0.5)
        marker.setColor(1., 0., 0., 1.)
        return marker

    def line_model(self, r, g, b):
        line = LineNodePath(self.render2d, 'box', 2)
        line.reparentTo(self.render)
        line.setColor(r, g, b, 1.)
        line.setTransparency(TransparencyAttrib.MAlpha)
        line.setAlphaScale(0.5)
        return line

    def point_model(self, position, color):
        marker = self.loader.loadModel("resources/models_3d/sphere")
        marker.reparentTo(self.render)
        marker.setScale(5, 5, 5)
        marker.setPos(position[0], position[1], position[2])
        marker.setTransparency(TransparencyAttrib.MAlpha)
        marker.setAlphaScale(0.5)
        marker.setColor(color[0], color[1], color[2], 1.)

    def line_draw(self, line, from_position, to_position):
        line.reset()
        line.drawLines([[(from_position[0], from_position[1], from_position[2]), (to_position[0], to_position[1], to_position[2])]])
        line.create()

    def get_dimensions(self):
        return [self.room_dimentions[0]/100., self.room_dimentions[1]/100.]

    def get_drone(self):
        return self.drone_instance

    def set_markers(self, markers):
        self.markers = []
        for marker in markers:
            self.markers.append(self.marker_model(self.convert_position(marker[0]), marker[1]))

    def set_default_markers(self):
        dimensions = self.get_dimensions()

        self.set_markers([
            [[dimensions[0] / 2, 1.5, 0.01], [0, 0, 0]],
            [[dimensions[0] / 2, 1.5, dimensions[1] - 0.01], [0, 0, 0]],
            [[dimensions[0] - 0.01, 1.5, dimensions[1] / 2], [90, 0, 0]],
            [[0.01, 1.5, dimensions[1] / 2], [90, 0, 0]]
        ])

    def get_markers(self):
        return self.markers

    def convert_position(self, position):
        return [position[0] * 100, position[2] * 100, position[1] * 100]

    def hook_init(self, callback):
        callback(self)

    def hook_loop(self, callback):
        self.loop_callback = callback


if __name__ == '__main__':
    app = World()
    app.run()