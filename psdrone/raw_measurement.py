import time
import ps_drone
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import xbox
from orientation import *

if __name__ == "__main__":
    drone = ps_drone.Drone()
    drone.debug = True
    drone.startup()
    drone.reset()

    while (drone.getBattery()[0] == -1): time.sleep(0.1)  # Reset completed ?
    print "Battery:"+str(drone.getBattery()[0]) +" % "+str(drone.getBattery()[1])

    drone.useDemoMode(False)
    drone.addNDpackage(["all"])

    while not "raw_measures" in drone.NavData:
        pass

    joy = None
    try:
        joy = xbox.Joystick()
    except:
        pass

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
        gyro = drone.NavData["raw_measures"][1]
        accelerometer = drone.NavData["raw_measures"][0]
        gyro[0] = (gyro[0]-39)/100.
        gyro[1] = (gyro[1]-30)/100.
        gyro[2] = (gyro[2]+33)/100.
        accelerometer = list(map(lambda x: (x-2020.)/50., accelerometer))
        accelerometer2 = [accelerometer[1],accelerometer[0],accelerometer[2]]
        print gyro, accelerometer2
        read_data(
            gyro=gyro,  # x, y, z
            accelerometer=accelerometer2  # x, y, z
        )
        draw()

        pygame.display.flip()
        frames += 1

        if joy is not None:
            (roll, throttle) = joy.leftStick()
            (yaw, pitch) = joy.rightStick()
            drone.move(roll, pitch, throttle, yaw)

        # key = drone.getKey()
        # if key == " ":
        #     if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:
        #         drone.takeoff()
        #     else:
        #         drone.land()
        # if key == "x":
        #     drone.emergency()
        # elif key == "0":
        #     drone.hover()
        # elif key == "w":
        #     drone.moveForward()
        # elif key == "s":
        #     drone.moveBackward()
        # elif key == "a":
        #     drone.moveLeft()
        # elif key == "d":
        #     drone.moveRight()
        # elif key == "q":
        #     drone.turnLeft()
        # elif key == "e":
        #     drone.turnRight()
        # elif key == "7":
        #     drone.turnAngle(-10, 1)
        # elif key == "9":
        #     drone.turnAngle(10, 1)
        # elif key == "4":
        #     drone.turnAngle(-45, 1)
        # elif key == "6":
        #     drone.turnAngle(45, 1)
        # elif key == "1":
        #     drone.turnAngle(-90, 1)
        # elif key == "3":
        #     drone.turnAngle(90, 1)
        # elif key == "8":
        #     drone.moveUp()
        # elif key == "2":
        #     drone.moveDown()
        # elif key == "*":
        #     drone.doggyHop()
        # elif key == "+":
        #     drone.doggyNod()
        # elif key == "-":
        #     drone.doggyWag()


