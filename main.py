#!/usr/bin/env python3

import sys
from robot_library.robot import *
import cv2
import rospy
import numpy as np
from matplotlib import pyplot as plt
import math


deg2rad = math.radians
deg90 = math.pi/2
block_size = 0.55
velocity = .5


def right():
    turn(1)
def left():
    turn(-1)
def round_angle(a):
    return round(robot.getDirection() / deg90) * deg90
def turn(a):
    target = round_angle(a) + deg90 * a
    turn_angle(target)
def turn_angle(target_dir: float) -> float:
    """
    function turns robot at given angle in radians
    returns predicted position after turn (calculated position, real position may differ)
    """
    global robot

    # defining some constants
    MAX_TURN_SPEED = 0.3
    P_COEF = 0.3

    current_dir = robot.getDirection()
    # calculate target direction of robot after turn
    # target_dir = current_dir + add_deg_rad

    # calculate error of rotation (nobody knows how it works, but it does)
    e = (target_dir - current_dir + np.pi * 5) % (np.pi * 2) - (np.pi)

    # accepting threshold after turn is 0.01
    while abs(e - np.sign(e) * np.pi) > 0.01:
        current_dir = robot.getDirection()
        e = (target_dir - current_dir + np.pi * 5) % (np.pi * 2) - (np.pi)
        turn_speed = -(e - np.sign(e) * np.pi) * P_COEF + np.sign(e) * 0.1

        # limit our speed with MAX_TURN_SPEED bound
        turn_speed = np.sign(turn_speed) * np.maximum(np.abs(turn_speed), MAX_TURN_SPEED)
        # equivalent to bottom line
        # turn_speed = (turn_speed if turn_speed > -MAX_TURN_SPEED else -MAX_TURN_SPEED) if turn_speed < MAX_TURN_SPEED else MAX_TURN_SPEED

        robot.setVelosities(0, turn_speed)

        # some delay for don't overload computation
        robot.sleep(0.001)

    robot.setVelosities(0,0)
    return target_dir


def std(x):
    return (x + np.pi * 5) % (np.pi * 2) - (np.pi)
def get_points(laser, rot=0., arm=0.455/2):
    angle = laser["angle"]
    laser = laser["values"]
    xs = np.linspace(-angle/2, +angle/2, len(laser)) + rot
    ys = np.array(laser)
    # mid = len(laser) // 2
    # print(ys[mid])
    # ys[mid-1] = 2.5
    # ys[mid] = 0
    xs, ys = np.cos(xs) * ys + np.cos(rot) * arm, np.sin(xs) * ys + np.sin(rot) * arm
    xs, ys = xs[40:-40], ys[40:-40]
    xy = np.stack((xs, ys), axis=-1)
    d = xy[1:] - xy[:-1]
    r = std(std(np.arctan2(d[:, 1], d[:, 0])) - rot)
    r = r[1:] - r[:-1]
    # r = r ** 2
    plt.plot(r)
    plt.show()
    return xs, ys


def m2cpr(x):
    return x / 0.09
def forward(controller=1):
    # compass
    target_angle = round_angle(robot.getDirection())
    def compass_error():
        err = (robot.getDirection() - target_angle + np.pi * 5) % (np.pi * 2) - (np.pi)
        return err * 50
    # laser
    sl, _, sr = sensor()
    def laser_error():
        l, _, r = sensor()
        if l < 0.5 and r < 0.5:
            return (r - l) * 50
        elif l < 0.5:
            return (l - 0.3) * 50
        elif r < 0.5:
            return (r - 0.3) * 50
        else:
            return compass_error()

    if controller == 0:
        error = compass_error
    elif controller == 1:
        error = laser_error

    # terminator
    encoder_start = robot.getEncoders()["left"]
    def termination():
        encoder_current = robot.getEncoders()["left"]
        _, f, _ = sensor()
        return (encoder_current - encoder_start >= m2cpr(block_size)) # or (f < 0.7)

    # PID
    pid(error, termination)
def pid(error, termination, vel=velocity, rot=1., sleep=0.005, kp=0.01, ki=0.0001, kd=0.03,):
    old_error = 0.
    I = 0
    while not termination():
        err = error()
        P = err * kp
        I += err * ki
        D = (err - old_error) * kd
        u = P + I + D
        robot.setVelosities(vel, u * rot)
        old_error = err
        robot.sleep(sleep)
    robot.setVelosities(0, 0)


def away(digits):
    for i in digits:
        if i == 0:
            pass
        elif i == 1:
            left()
        elif i == 2:
            # bft()
            right()
        elif i == 3:
            forward()
            # if sF() < 30: bff()
        robot.sleep(0.1)
def sensor():
    laser = robot.getLaser()
    vals = laser["values"]
    angle = laser["angle"] / 2
    x = np.linspace(-angle, +angle, len(vals))
    closest = lambda k: vals[np.argmin(np.abs(x - k))]
    point = np.pi/2
    return closest(point), vals[len(vals) // 2], closest(-point)
def rhr():
    l, f, r = sensor()
    if r > 0.5:
        print("right+forward")
        right()
        forward()
    elif f < 0.65:
        print("obstacle+", end='')
        if l > 0.5:
            print("left")
            left()
        else:
            print("right")
            right()
        forward()
    else:
        print("forward")
        forward()



if __name__ == "__main__":

    # initialize robot
    robot = Robot()

    # exit()
    # right()
    # left()

    while True:
        rhr()
        robot.sleep(0.1)
    exit()

    away([3, 3, 3, 3,  2,
          3, 3, 3, 3,  2,
          3, 3, 2,
          3, 3, 2,
          3, 3, 1,
          3, 3, 2])
    exit()

    #
    # print(robot.getEncoders())
    # robot.setVelosities(0.5, 0)
    # last = NoneFigure_1.png
    # for _ in range(8):
    #     robot.sleep(0.25)
    #     encoder = robot.getEncoders()["right"]
    #     laser = robot.getLaser()["values"]
    #     laser = laser[len(laser) // 2]
    #     if last is not None:
    #         print(encoder - last[0], laser - last[1], (encoder - last[0]) / (laser - last[1]))
    #     last = encoder, laser
    # robot.setVelosities(0, 0)
    # exit()

    # robot.setVelosities(2, 0)
    # robot.sleep(1)
    # robot.setVelosities(0, 0)
    # exit()

    print(robot.getDirection())
    # for _ in range(20):
    #     robot.setVelosities(0, -0.8)
    #     robot.sleep(0.05)
    robot.setVelosities(0, 0)

    # get laser values

    laser1 = robot.getLaser()
    dir1 = robot.getDirection()
    print('l1')
    xy1 = get_points(laser1, dir1)
    plt.clf()

    plt.axes().set_aspect('equal')
    plt.plot(*xy1)
    plt.show()
    plt.clf()

    robot.setVelosities(0.6, 0.7)
    robot.sleep(1)
    robot.setVelosities(0, 0)

    laser2 = robot.getLaser()
    dir2 = robot.getDirection()

    xy2 = get_points(laser2, dir2)
    plt.clf()
    plt.axes().set_aspect('equal')
    plt.plot(*xy2)
    plt.show()

    exit()
    while True:
        robot.setVelosities(0, -10.0)
        for _ in range(10):
            robot.sleep(0.5)
            print(robot.getDirection())
        robot.setVelosities(0, 0)

        print(robot.getDirection())
        robot.sleep(0.5)

        print()
        print(robot.getDirection())
        print()


        robot.setVelosities(0, -0.3)
        for _ in range(10):
            robot.sleep(0.5)
            print(robot.getDirection())
        robot.setVelosities(0, 0)
        print()
        robot.sleep(1)

        robot.setVelosities(0, -0.3)
        for _ in range(10):
            robot.sleep(0.5)
            print(robot.getDirection())
        robot.setVelosities(0, 0)
        print()
        robot.sleep(5)

    # get values of laser and remove some extreme values because laser see himself
    laser = laser.get('values')[40:len(laser.get('values'))-40]
    print(laser)

    # how close robot could go to the obstacles
    threshold = 0.5

    robot.setVelosities(0.3, 0)
    wind = 40

    while True:

        robot.setVelosities(0.3, -0.1)
        laser = robot.getLaser()
        length = len(laser.get('values'))
        laser = laser.get('values')[length//2-wind:length//2+wind]
        print(laser)

        # find minimum value if laser values
        min = 5
        for i in laser:
            if i < min:
                min = i
        print("h = %s"%(min))

        # Check should or should not robot turn to avoid obstacle
        if (min < threshold):
            print("There's an obstacle near")
            robot.setVelosities(0,0.3)
            while(min <= threshold):
                laser = robot.getLaser()
                laser = laser.get('values')[length//2-wind:length//2+wind]
                min = 5
                for i in laser:
                    if i < min:
                        min = i
                print("h = %s" % (min))
                robot.sleep(0.5)

            print("no wall")
            robot.setVelosities(0.3, 0)

        # some delay for don't overload computation
        robot.sleep(0.005)
