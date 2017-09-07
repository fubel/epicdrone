import time
import ps_drone
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import xbox

if __name__ == "__main__":
    drone = ps_drone.Drone()
    drone.debug = True
    drone.startup()
    drone.reset()

    while (drone.getBattery()[0] == -1): time.sleep(0.1)  # Reset completed ?
    print "Battery:"+str(drone.getBattery()[0]) +" % "+str(drone.getBattery()[1])

    drone.useDemoMode(False)
    drone.addNDpackage(["demo"])
    drone.addNDpackage(["all"])

    while not "raw_measures" in drone.NavData:
        pass
    joy = None
    try:
        joy = xbox.Joystick()
    except:
        pass

    fig = plt.figure()
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2,2,3, projection='3d')
    data_accelerometer = [[],[],[]]
    data_gyrometer = [[],[],[]]

    def graph_loop(index):
        for i in range(len(data_accelerometer)):
            if len(data_accelerometer[i]) > 100:
                data_accelerometer[i].pop(0)
            data_accelerometer[i].append(drone.NavData["raw_measures"][0][i])
        for i in range(len(data_gyrometer)):
            if len(data_gyrometer[i]) > 100:
                data_gyrometer[i].pop(0)
            data_gyrometer[i].append(drone.NavData["raw_measures"][1][i])

        ax1.clear()
        ax1.axis([0, 100, -3000, 3000])
        ax2.clear()
        ax2.axis([0, 100, -3000, 3000])
        ax3.clear()
        ax3.set_xlim3d(-200, 200)
        ax3.set_ylim3d(-200,200)
        ax3.set_zlim3d(-200,200)
        for i in range(len(data_accelerometer)):
            ax1.plot(data_accelerometer[i])
        for i in range(len(data_gyrometer)):
            ax2.plot(data_gyrometer[i])
        ax3.plot([0,drone.NavData["magneto"][0][0]], [0,drone.NavData["magneto"][0][1]], [0,drone.NavData["magneto"][0][2]])

        if joy is not None:
            (roll, throttle) = joy.leftStick()
            (yaw, pitch) = joy.rightStick()
            drone.move(roll, pitch, throttle, yaw)

        key = drone.getKey()
        print str(drone.NavData["demo"][0][2]) + " " + key + " " + str(drone.NavData["magneto"][10]) + " " + str(drone.NavData["magneto"][11])
        if key == " ":
            if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:
                drone.takeoff()
            else:
                drone.land()
        if key == "x":
            drone.emergency()
        elif key == "0":
            drone.hover()
        elif key == "w":
            drone.moveForward()
        elif key == "s":
            drone.moveBackward()
        elif key == "a":
            drone.moveLeft()
        elif key == "d":
            drone.moveRight()
        elif key == "q":
            drone.turnLeft()
        elif key == "e":
            drone.turnRight()
        elif key == "7":
            drone.turnAngle(-10, 1)
        elif key == "9":
            drone.turnAngle(10, 1)
        elif key == "4":
            drone.turnAngle(-45, 1)
        elif key == "6":
            drone.turnAngle(45, 1)
        elif key == "1":
            drone.turnAngle(-90, 1)
        elif key == "3":
            drone.turnAngle(90, 1)
        elif key == "8":
            drone.moveUp()
        elif key == "2":
            drone.moveDown()
        elif key == "*":
            drone.doggyHop()
        elif key == "+":
            drone.doggyNod()
        elif key == "-":
            drone.doggyWag()

    loop = animation.FuncAnimation(fig, graph_loop, interval=20)
    plt.show()

