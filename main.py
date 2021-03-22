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
bff_limit = 0.15
bff_vel = 0.25


def right():
    turn(1)
def left():
    turn(-1)
def round_angle(a):
    return round(robot.getDirection() / deg90) * deg90
def turn(a):
    target = round_angle(a) + deg90 * a
    turn_angle(target)
def turn_angle(target_dire: float) -> float:
    """
    function turns robot at given angle in radians
    returns predicted position after turn (calculated position, real position may differ)
    """
    global robot

    # defining some constants
    MAX_TURN_SPEED = 0.4
    P_COEF = 0.4

    current_dir = robot.getDirection()
    # calculate target direction of robot after turn
    target_dir = target_dire + current_dir # + add_deg_rad

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
    angle = np.pi * 4 / 3 # laser["angle"]
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
    return xy
    d = xy[1:] - xy[:-1]
    r = std(std(np.arctan2(d[:, 1], d[:, 0])) - rot)
    r = r[1:] - r[:-1]
    source = np.cos(np.linspace(-angle/2, +angle/2, len(laser)))[40:-40][2:]
    return source, r
    # r = r ** 2
    plt.plot(r)
    plt.show()
    return xs, ys


def m2cpr(x):
    return x / 0.09
def compass_error():
    # compass
    target_angle = round_angle(robot.getDirection())
    def error():
        err = (robot.getDirection() - target_angle + np.pi * 5) % (np.pi * 2) - (np.pi)
        return err * 50
    return error
def laser_error():
    # laser
    sl, _, sr = sensor()
    compass = compass_error()
    def error():
        l, _, r = sensor()
        if l < 0.5 and r < 0.5:
            return (r - l) * 150
        elif l < 0.5:
            return (0.28 - l) * 150
        elif r < 0.5:
            return (r - 0.28) * 150
        else:
            return compass()
    return error
def forward(controller=1):
    if controller == 0:
        error = compass_error()
    elif controller == 1:
        error = laser_error()

    # terminator
    encoder_start = robot.getEncoders()["left"]
    _, laser_start, _ = sensor()
    def termination():
        encoder_current = robot.getEncoders()["left"]
        _, f, _ = sensor()
        return (encoder_current - encoder_start >= m2cpr(block_size)) or (f < bff_limit)

    # PID
    pid(error, termination)
def bff():
    error = laser_error()
    _, sf, _ = sensor()
    if 0.6 < sf < 0.7:
        # print(sf)
        # print("shwup", end='')
        robot.setVelosities(-0.5, 0)
        robot.sleep(0.5)
        robot.setVelosities(0, 0)
    if sf > bff_limit:
        goal = 1
        def termination():
            _, f, _ = sensor()
            if f <= bff_limit:
                return True
            return False
    else:
        goal = -1
        def termination():
            _, f, _ = sensor()
            if f >= bff_limit:
                 #print("term")
                return True
            # print("non")
            return False

    pid(error, termination, vel=bff_vel*goal)


def pid(error, termination, vel=velocity, rot=1., sleep=0.0025, kp=0.01, ki=0.0001, kd=0.03,):
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
    vals = None
    for _ in range(3):
        laser = robot.getLaser()
        if vals is None:
            vals = np.array(laser["values"])
        else:
            vals += np.array(laser["values"])
    # vals = laser["values"]
    vals /= 3
    angle = laser["angle"] / 2
    x = np.linspace(-angle, +angle, len(vals))
    closest = lambda k: vals[np.argmin(np.abs(x - k))]
    point = np.pi/2
    return closest(point), min(vals[len(vals) // 2-8:len(vals) // 2+8]), closest(-point)
def rhr():
    l, f, r = sensor()
    if r > 0.5:
        print("right+forward")
        right()
        forward()
    elif f < 0.7:
        print("obstacle+bff+", end='')
        bff()
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


def sanitize(a):
    return a[np.all(np.isfinite(a) & np.isfinite(b), axis=-1)].copy()


def icp(a, b):
    a, b = a[np.all(np.isfinite(a) & np.isfinite(b), axis=-1)].copy(), b[np.all(np.isfinite(b) & np.isfinite(a), axis=-1)].copy()
    sqrs = np.sum((a - b) ** 2, axis=-1)
    keep = sqrs < np.sort(sqrs)[int(len(sqrs) * 0.5)]
    # a, b = a[keep], b[keep]

    a_mean = np.mean(a, axis=0)
    b_mean = np.mean(b, axis=0)
    # a -= a_mean
    # b -= b_mean
    n = a.T @ b
    u, d, vt = np.linalg.svd(n)
    rot = np.dot(vt.T, u.T)
    if np.linalg.det(rot) < 0:
        vt[1, :] *= -1
        rot = vt.T @ u.T
    t = b_mean.T - rot @ a_mean.T
    # t = b_mean - a_mean # np.mean(b - a, axis=0)
    mat = np.identity(3)
    # mat[:2, :2] = rot
    mat[:2, 2] = np.mean(a - b, axis=0)  # b_mean - a_mean
    return mat, rot, t


def kn():
    knn = cv2.ml.KNearest_create()
    knn.train(points_a.astype(np.float32), cv2.ml.ROW_SAMPLE, np.array(list(range(len(points_a))), dtype=np.int32))

    m = np.identity(3)
    ds = []
    points_d = np.concatenate((points_b, np.ones((points_b.shape[0], 1))), axis=-1).copy()
    for i in range(150):
        print(i)
        print("neighbors")
        _, result, _, _ = knn.findNearest(points_d.astype(np.float32)[:, :2], k=1)
        result = result[:, 0].astype(np.int32)
        points_c = points_a[result]
        print("icp")
        m_, r, t = icp(points_d[:, :2], points_c)
        m = np.matmul(m, m_)
        print("plot")
        points_d = np.einsum("ij, ki -> kj", m_, points_d)
        points_d = np.concatenate((points_d[:, :2], np.ones((points_d.shape[0], 1))), axis=-1).copy()
        ds.append(points_d)

    for points_d in ds[-1:]:
        plt.plot(points_d[:, 0], points_d[:, 1])


if __name__ == "__main__":

    # initialize robot
    robot = Robot()

    # help(cv2.ppf_match_3d)
    # exit()

    robot.setVelosities(0., -.6)
    robot.sleep(2.)
    points_a = get_points(robot.getLaser(), robot.getDirection())

    robot.setVelosities(.5, -.6)
    robot.sleep(1.)
    points_b = get_points(robot.getLaser(), robot.getDirection())
    robot.setVelosities(0, 0)
    points_b += np.mean(points_a, axis=0) - np.mean(points_b, axis=0)

    plt.axes().set_aspect('equal')
    plt.plot(points_a[:, 0], points_a[:, 1])
    plt.plot(points_b[:, 0], points_b[:, 1])
    plt.show()


    exit()


    # left()
    print('a')
    print('b')
    laser = robot.getLaser()
    img = robot.getImage()
    img = img[img.shape[0] // 2:-85]


    cv2.imshow("hey", img)
    cv2.waitKey(0)

    plt.plot(laser["values"][::-1])
    plt.show()
    exit()
    x, y = get_points(laser, robot.getDirection())
    plt.plot(x, np.abs(y))
    plt.show()
    exit()

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
