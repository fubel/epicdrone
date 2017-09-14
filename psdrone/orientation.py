from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import numpy as np


ax = ay = az = 0.0
gx = gy = gz = 0.0


def resize((width, height)):
	if height==0:
		height=1
	glViewport(0, 0, width, height)
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective(45, 1.0*width/height, 0.1, 100.0)

	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()

def init():
	glShadeModel(GL_SMOOTH)
	glClearColor(0.0, 0.0, 0.0, 0.0)
	glClearDepth(1.0)
	glEnable(GL_DEPTH_TEST)
	glDepthFunc(GL_LEQUAL)
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def drawText(position, textString):
	font = pygame.font.SysFont ("Courier", 18, True)
	textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))
	textData = pygame.image.tostring(textSurface, "RGBA", True)
	glRasterPos3d(*position)
	glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def draw():
	global rquad
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

	glLoadIdentity()
	glTranslatef(0, 0.0, -7.0)

	osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax)) + ", yaw: " + str("{0:.2f}".format(az))

	drawText((-2, -2, 2), osd_text)

	glBegin(GL_LINES)
	glColor3f(0.0, 1.0, 0.0)
	glVertex3f(0.0, 0.0, 0.0)
	glVertex3f(15., 0., 0.)

	glColor3f(1.0, 1.0, 0.0)
	glVertex3f(0.0, 0.0, 0.0)
	glVertex3f(0., 15., 0.)

	glColor3f(1.0, .0, 1.0)
	glVertex3f(0.0, 0.0, 0.0)
	glVertex3f(0., 0., 15.)

	glEnd()

	glRotatef(az, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
	glRotatef(ay, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
	glRotatef(-1 * ax, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis

	glBegin(GL_QUADS)
	glColor3f(0.0, 1.0, 0.0)
	glVertex3f(1.0, 0.2, -1.0)
	glVertex3f(-1.0, 0.2, -1.0)
	glVertex3f(-1.0, 0.2, 1.0)
	glVertex3f(1.0, 0.2, 1.0)

	glColor3f(1.0, 0.5, 0.0)
	glVertex3f(1.0, -0.2, 1.0)
	glVertex3f(-1.0, -0.2, 1.0)
	glVertex3f(-1.0, -0.2, -1.0)
	glVertex3f(1.0, -0.2, -1.0)

	glColor3f(1.0, 0.0, 0.0)
	glVertex3f(1.0, 0.2, 1.0)
	glVertex3f(-1.0, 0.2, 1.0)
	glVertex3f(-1.0, -0.2, 1.0)
	glVertex3f(1.0, -0.2, 1.0)

	glColor3f(1.0, 1.0, 0.0)
	glVertex3f(1.0, -0.2, -1.0)
	glVertex3f(-1.0, -0.2, -1.0)
	glVertex3f(-1.0, 0.2, -1.0)
	glVertex3f(1.0, 0.2, -1.0)

	glColor3f(0.0, 0.0, 1.0)
	glVertex3f(-1.0, 0.2, 1.0)
	glVertex3f(-1.0, 0.2, -1.0)
	glVertex3f(-1.0, -0.2, -1.0)
	glVertex3f(-1.0, -0.2, 1.0)

	glColor3f(1.0, 0.0, 1.0)
	glVertex3f(1.0, 0.2, -1.0)
	glVertex3f(1.0, 0.2, 1.0)
	glVertex3f(1.0, -0.2, 1.0)
	glVertex3f(1.0, -0.2, -1.0)
	glEnd()


def read_data(gyro, accelerometer):
	global ax, ay, az
	global gx, gy, gz

	ax = ay = az = 0.0

	# angles based on accelerometer
	ay = np.arctan2(accelerometer[1], np.sqrt(pow(accelerometer[0], 2) + pow(accelerometer[2], 2))) * 180. / np.pi
	ax = np.arctan2(accelerometer[0], np.sqrt(pow(accelerometer[1], 2) + pow(accelerometer[2], 2))) * 180. / np.pi
	print ax, ay
	# angles based on gyro(deg / s)
	gx += gyro[0] / 8
	gy -= gyro[1] / 8
	gz += gyro[2] / 8

	# weighting both measurments
	gx = gx * 0.96 + ax * 0.04
	gy = gy * 0.96 + ay * 0.04

	ax = gx
	ay = gy
	az = gz


def get_drone_orientation(drone):
	read_data(
		gyro=[drone.NavData["raw_measures"][1]],  # x, y, z
		accelerometer=[drone.NavData["raw_measures"][0]]  # x, y, z
	)
	return ax, ay, az


if __name__ == '__main__':
	video_flags = OPENGL | DOUBLEBUF

	pygame.init()
	screen = pygame.display.set_mode((640, 480), video_flags)
	pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
	resize((640, 480))
	init()
	frames = 0
	ticks = pygame.time.get_ticks()
	while True:
		event = pygame.event.poll()
		if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
			break
		read_data(
			gyro=[0., 0., 0.],  # x, y, z
			accelerometer=[6.8, 5.3, 6.8]  # x, y, z
		)
		draw()

		pygame.display.flip()
		frames += 1

	print "fps:  %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks))
