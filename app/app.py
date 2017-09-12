from panda3d.core import loadPrcFile
loadPrcFile("config/Config.prc")

from psdrone.simulation import *

import numpy as np
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import TransparencyAttrib
from direct.gui.OnscreenText import OnscreenText, TextNode
from direct.directtools.DirectGeometry import LineNodePath

D = DummyDrone(np.array([3.5, 1., 2.]), 0)
K = Kalman1D(D)

movements = [np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]),
			 np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]),
			 np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]), np.array([+1.0, 0, 0]),
			 np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0]), np.array([-1.0, 0, 0])]

# mesh for kalman gaussian vis
x = np.linspace(D.x_range[0], D.x_range[1], 500)


class MyApp(ShowBase):

	fps_text = fps_text2 = fps_text3 = fps_text4 = None
	camera_position = [1468, 1177, 1160, -126, -38, 0]  # x y z h p r
	drone_position = [200, 400, 150, 0, 0, 0]  # x y z h p r
	markers = {}
	marker_lines = {}
	active_keys = {}

	def __init__(self):
		ShowBase.__init__(self)

		#self.useDrive()

		# Load the environment model.
		self.wall1 = self.wall_model(0, 0, 0, alpha=True)
		self.wall2 = self.wall_model(0, 0, 0, alpha=True).setH(90)
		self.wall3 = self.wall_model(800, 800, 0).setH(180)
		self.wall4 = self.wall_model(800, 800, 0).setH(-90)
		#self.marker1 = self.marker_model(1, 400, 150, 90, 0, 0)
		#self.marker3 = self.marker_model(400, 1, 150, 0, 0, 0)
		#self.marker4 = self.marker_model(799, 400, 150, 90, 0, 0)
		#self.marker2 = self.marker_model(400, 799, 150, 0, 0, 0)
		for key, value in D.landmarks.iteritems():
			self.markers[key] = self.marker_model(D.landmarks[key][0]+100, D.landmarks[key][1]+100, D.landmarks[key][2]+100, 90, 0, 0)
			self.marker_lines[key] = self.line_model(1., 1., 1.)
		self.l1 = self.line_model(0., 1., 0.)
		self.l2 = self.line_model(1., 0., 0.)
		self.l3 = self.line_model(0., 0., 1.)
		self.l4 = self.line_model(0., 1., 1.)
		self.l5 = self.line_model(1., 1., 0.)

		# Load and transform the panda actor.
		self.drone = self.drone_model()

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
			self.camera_position[0] +=  10 * np.sin(np.deg2rad(self.camera_position[3])) * parameter[1]
			self.camera_position[1] +=  10 * np.cos(np.deg2rad(self.camera_position[3])) * parameter[1]
		if parameter[0] == "z":
			self.camera_position[2] += 10 * parameter[1]
		if parameter[0] == "h":
			self.camera_position[3] += parameter[1]
		if parameter[0] == "p":
			self.camera_position[4] += parameter[1]

	def move_drone(self, axis, direction):
		print axis, direction
		if axis == "x":
			self.drone_position[0] += 5 * direction
		if axis == "y":
			self.drone_position[1] += 5 * direction
		if axis == "z":
			self.drone_position[2] += 1 * direction
		if axis == "h":
			self.drone_position[3] += direction

	def tick(self, task):
		self.camera.setPos(self.camera_position[0], self.camera_position[1], self.camera_position[2])
		self.camera.setHpr(-self.camera_position[3], self.camera_position[4], self.camera_position[5])

		for key in self.active_keys:
			if self.active_keys[key] is not None:
				self.active_keys[key][0](self.active_keys[key][1])

		if task.frame%60 == 0:
			for movement in movements + movements[::-1]:
				time_step = 1
				K.predict(movement)
				m = D.measure()
				K.update(m)
				D.update(m)
				self.drone.setPos(D.real_position[0]*100, D.real_position[1]*100, D.real_position[2]*100)
				self.drone.setHpr(-self.drone_position[3], self.drone_position[4], self.drone_position[5])
				#ax.scatter(D.estimated_position[0], D.estimated_position[1],
				#		   D.estimated_position[2], zdir='y', c='gray', label='Average estimation')
				#ax.scatter(K.state[0], D.real_position[1],
				#		   D.real_position[2], zdir='y', c='green', label='Kalman estimation')
				#rv = norm(loc=K.state[0], scale=K.state[1])
				#ax.plot(x, rv.pdf(x), zdir='y', label='Kalman Filter belief')
				dir = np.array([0, 0, 1], dtype=np.float32)
				dir = 1 / (2 * np.linalg.norm(dir)) * dir
				print dir
				self.line_draw(self.l1,
							   [0, 0, 0],
							   [D.real_position[0]*100, D.real_position[1]*100, D.real_position[2]*100])
				self.line_draw(self.l2,
							   [D.real_position[0]*100, D.real_position[1]*100, D.real_position[2]*100],
							   [(D.real_position[0] + dir[0])*100, (D.real_position[1] + dir[1])*100, (D.real_position[2] + dir[2])*100])
				self.line_draw(self.l3,
							   [0, 0, 0],
							   [D.estimated_position[0]*100, D.estimated_position[1]*100, D.estimated_position[2]*100])
				self.line_draw(self.l4,
							[D.estimated_position[0] * 100, D.estimated_position[1] * 100, D.estimated_position[2] * 100],
							[(D.estimated_position[0] + dir[0]) * 100, (D.estimated_position[1] + dir[1]) * 100, (D.estimated_position[2] + dir[2]) * 100])

				for key, value in D.landmarks.iteritems():
					self.line_draw(self.marker_lines[key],
								   [0,0,0],
								   [D.landmarks[key][0]*100, D.landmarks[key][1]*100, D.landmarks[key][2]*100])
					#ax.scatter(D.landmarks[key][0], D.landmarks[key][1], D.landmarks[key][2], zdir='y', c='blue')
					# arw2 = Arrow3D.arrow(np.array([0, 0, 0]), D.landmarks[key], color='gray')
					# ax.add_artist(arw2)
				#	if key in D.observable_landmarks():
				#		arw3 = Arrow3D.arrow(D.landmarks[key], D.real_position)
				#		ax.add_artist(arw3)
				textstr = '$MSE_A = $%s, $MSE_K = $%s' % (D.error, (D.real_position[0] - K.state[0]) ** 2)

				if self.fps_text4 is not None:
					self.fps_text4.destroy()
				self.fps_text4 = OnscreenText(text='$MSE_A = $%s, $MSE_K = $%s' % (D.error, (D.real_position[0] - K.state[0]) ** 2),
										 pos=(0.05, 0.20),
										 scale=0.05,
										 align=TextNode.ALeft,
										 parent=self.a2dBottomLeft)

		if task.time > 0:
			if self.fps_text is not None:
				self.fps_text.destroy()
			if self.fps_text2 is not None:
				self.fps_text2.destroy()
			if self.fps_text3 is not None:
				self.fps_text3.destroy()
			self.fps_text = OnscreenText(text="Tick-Rate: " + str(int(task.frame / task.time)),
										 pos=(0.05, 0.05),
										 scale=0.05,
										 align=TextNode.ALeft,
										 parent=self.a2dBottomLeft)
			self.fps_text2 = OnscreenText(text="Camera Position: " + str(self.camera_position),
										 pos=(0.05, 0.1),
										 scale=0.05,
										 align=TextNode.ALeft,
										 parent=self.a2dBottomLeft)
			self.fps_text3 = OnscreenText(text="Drone Position: " + str(D.real_position),
										 pos=(0.05, 0.15),
										 scale=0.05,
										 align=TextNode.ALeft,
										 parent=self.a2dBottomLeft)

		return Task.cont

	def drone_model(self):
		drone = self.loader.loadModel("models/box")
		drone.setScale(30, 30, 20)
		drone.reparentTo(self.render)
		return drone

	def wall_model(self, x, y , z, alpha = False):
		wall = self.loader.loadModel("models/plane")
		wall.reparentTo(self.render)
		wall.setScale(800, 0, 300)
		wall.setPos(x, y, z)
		wall.setTransparency(TransparencyAttrib.MAlpha)
		wall.setAlphaScale(0.1)
		return wall

	def marker_model(self, x, y, z, h, p, r):
		marker = self.loader.loadModel("models/plane")
		marker.reparentTo(self.render)
		marker.setScale(21, 0, 21)
		marker.setPos(x, y, z)
		marker.setHpr(h, p, r)
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

	def line_draw(self, line, from_position, to_position):
		line.reset()
		line.drawLines([[(from_position[0], from_position[1], from_position[2]), (to_position[0], to_position[1], to_position[2])]])
		line.create()


app = MyApp()
app.run()