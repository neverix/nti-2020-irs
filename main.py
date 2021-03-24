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

    current_dir = std(robot.getDirection())
    # calculate target direction of robot after turn
    target_dir = std(target_dire) # + current_dir # + add_deg_rad

    # calculate error of rotation (nobody knows how it works, but it does)
    e = target_dir - current_dir
    # print(current_dir, target_dir, target_dire)

    # accepting threshold after turn is 0.01
    while abs(e) > 0.01:
        current_dir = std(robot.getDirection())
        e = target_dir - current_dir
        # print(e)
        if abs(e) < 0.1:
            mts = .05
        else:
            mts = MAX_TURN_SPEED
        turn_speed = (e - np.sign(e) * np.pi) * P_COEF + np.sign(e) * 0.1

        # limit our speed with MAX_TURN_SPEED bound
        # turn_speed = np.sign(turn_speed) *
        # equivalent to bottom line
        turn_speed = (turn_speed if turn_speed > -mts else -mts) if turn_speed < mts else mts

        robot.setVelosities(0, turn_speed)

        # some delay for don't overload computation
        robot.sleep(0.001)

    robot.setVelosities(0,0)
    return target_dir


def std(x):
    if x < 0:
        x = 2 * np.pi + x
    return x % (np.pi * 2)
def get_points(laser, rot=0., arm=0.# -.455/2# .455/4#3
               ):
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
    dist = lambda a, b: (a - b) ** 2# , axis=-1)
    diff = lambda r: np.arctan2((np.roll(xy, r) - xy)[:, 1], (np.roll(xy, r) - xy)[:, 0])
    df = diff(1)
    dists_fw = dist(np.roll(df, 1) - df, np.roll(df, 2) - np.roll(df, 1))
    db = diff(1)
    dists_back = dist(np.roll(db, -1) - db, np.roll(db, -2) - np.roll(db, -1))
    dists = np.maximum(dists_back, dists_fw)
    start = dists[0]
    end = None
    totals = []
    for i, (d, point) in enumerate(zip(dists[1:], xy[1:])):
        if d > .2:
            end = point
            totals.append((start, end))
            end = start = None
        if start is None:
            start = point
    corr = []
    for start, end in totals:
        corr.append(np.linspace(start, end, num=50))
    return np.concatenate(corr, axis=0)
    exit()
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
    target_angle = robot.getDirection() # round_angle(robot.getDirection())
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
def forward(controller=1, bs=block_size):
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
        return (encoder_current - encoder_start >= m2cpr(bs)) or (f < bff_limit)

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


def sanitize(a, b):
    return a[np.all(np.isfinite(a) & np.isfinite(b), axis=-1)].copy()



def icp2(a, b):
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


def icp(a, b, keep_thresh=.75):
    a, b = a[np.all(np.isfinite(a) & np.isfinite(b), axis=-1)].copy(), b[np.all(np.isfinite(b) & np.isfinite(a), axis=-1)].copy()
    sqrs = np.sum((a - b) ** 2, axis=-1)
    # print(np.sort(sqrs)[int(len(sqrs) * keep_thresh)])
    keep = sqrs < np.sort(sqrs)[int(len(sqrs) * keep_thresh)]
    a, b = a[keep], b[keep]
    a_mean = np.mean(a, axis=0)
    b_mean = np.mean(b, axis=0)
    a = a - a_mean
    b = b - b_mean
    n = np.dot(a.T, b)
    u, d, vt = np.linalg.svd(n)
    rot = np.dot(vt.T, u.T)
    if np.linalg.det(rot) < 0:
        vt[1, :] *= -1
        rot = np.dot(vt.T, u.T)
    # rot = np.identity(2)
    t = b_mean.T - np.dot(rot, a_mean.T)
    mat = np.identity(3)
    mat[:2, :2] = rot
    mat[:2, 2] = t
    return mat, rot, t


def kn():
    knn = cv2.ml.KNearest_create()
    knn.train(points_a.astype(np.float32), cv2.ml.ROW_SAMPLE, np.array(list(range(len(points_a))), dtype=np.int32))

    m = np.identity(3)
    ds = []
    points_d = points_b.copy()#np.concatenate((points_b, np.ones((points_b.shape[0], 1))), axis=-1).copy()
    for i in range(150):
        print(i)
        print("neighbors")
        _, result, _, _ = knn.findNearest(points_d.astype(np.float32)[:, :2], k=1)
        result = result[:, 0].astype(np.int32)
        points_c = points_a[result]
        print("icp")
        points_d = points_d + np.mean(points_c - points_d, axis=0)
        # m_, r, t = icp(points_d[:, :2], points_c)
        # m = np.matmul(m, m_)
        # print("plot")
        # points_d = np.einsum("ij, ki -> kj", m_, points_d)
        # points_d = np.concatenate((points_d[:, :2], np.ones((points_d.shape[0], 1))), axis=-1).copy()
        ds.append(points_d)
    for points_d in ds[-1:]:
        plt.plot(points_d[:, 0], points_d[:, 1])


def world2img(x):
    return x.copy() / 10. + .5


def create_img(origin, points, img_size=512, kernel=9):
    img = np.zeros((img_size, img_size), dtype=np.uint8)
    conv = lambda x: world2img(x) * img_size
    indices = conv(points)
    tile = img.copy()
    tuplify = lambda y: tuple(int(x) for x in y.flatten())
    origin = tuplify(conv(origin))
    for point in indices:
        if any(~np.isfinite(point)):
            continue
        cv2.line(tile, origin, tuplify(point), (255,), thickness=kernel)
    indices = indices[np.all(np.isfinite(indices), axis=1)].astype(np.int32)
    img[indices[:, 0], indices[:, 1]] = 1
    img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel)))
    return img, tile

def trans(img, x, y):
    return cv2.warpAffine(img, np.float32([
                    [1, 0, x],
                    [0, 1, y]
                ]), (img.shape[1], img.shape[0]))


def align(a, b, m=None, iterations=25, keep_thresh=.75):
    knn = cv2.ml.KNearest_create()
    a = a[np.any(np.isfinite(a), axis=-1)]
    knn.train(a.astype(np.float32), cv2.ml.ROW_SAMPLE, np.array(list(range(len(a))), dtype=np.float32)[:, None])

    if m is None:
        m = np.identity(3)
    source_m = np.identity(3)
    ds = []
    points_d = np.concatenate((b, np.ones((b.shape[0], 1))), axis=-1).copy()
    points_d = np.dot(m, points_d.T).T
    err = 0.
    for i in range(iterations):
        # print(i)
        # print("neighbors")
        _, result, neighbors, _ = knn.findNearest(points_d.astype(np.float32)[:, :2], k=1)
        result = result[:, 0].astype(np.int32)
        points_c = a[result]
        # print("icp")
        # points_d = points_d + np.mean(points_c - points_d, axis=0)
        m_, r, t = icp(points_d[:, :2], points_c, keep_thresh)
        # m = np.matmul(m, m_)
        # print("plot")
        points_d = np.dot(m_, points_d.T).T
        errs = points_d[:, :2] - points_c
        errs = errs[np.isfinite(errs)]
        err = np.sum(errs ** 2)
        m = np.dot(m, m_)
        source_m = np.dot(source_m, m_)
        # points_d = np.concatenate((points_d[:, :2], np.ones((points_d.shape[0], 1))), axis=-1).copy()
        ds.append(points_d)
    return points_d[:, :2], source_m, err


def keyframe():
    global img, m
    direc = robot.getDirection()
    # print(direc)
    for i in range(5):
        # print(i)
        robot.setVelosities(.0, .5)
        robot.sleep(.5)
        # robot.sleep(2.)
        # robot.setVelosities(.5, 0)
        # robot.sleep(2.)
        a, _, b = robot.getDirection(), robot.sleep(.001), robot.getDirection()
        points_b = get_points(robot.getLaser(), (a + b) / 2)
        robot.setVelosities(0, 0)
        # plt.plot(points_b[:, 0], points_b[:, 1])
        # plt.plot(points_last[:, 0], points_last[:, 1])
        points_c = np.dot(m, np.concatenate((points_b, np.ones((points_b.shape[0], 1))), axis=-1).copy().T).T

        # plt.plot(points_c[:, 0], points_c[:, 1])

        # points_d, m_, err = align(points_last, points_b, m, keep_thresh=.65)
        # plt.plot(points_d[:, 0], points_d[:, 1])

        # plt.show()
        m_, err, points_d = np.identity(3), 0., points_b
        # points_de = np.concatenate((points_d, np.ones((points_d.shape[0], 1))), axis=-1).copy()
        # points_de = np.dot(m_, points_de.T).T[:, :2]
        m = np.dot(m, m_)

        # print(err)
        origin = m[:2, 2]
        img_, tile_ = create_img(origin, points_d, 512, 5)
        img += img_
        cv2.imshow("img", ((img / img.max() > .1) * 255).astype(np.uint8))
        robot.setVelosities(0, 0)
        cv2.waitKey(1)
        # points_last = points_d[np.all(np.isfinite(points_d), axis=-1), :2]
    robot.setVelosities(0, 0)
    print(robot.getDirection())
    turn_angle(std(direc))
    print(robot.getDirection())



def rem(key=True, plot=False, draw=False):
    global m, img, tile
    points = all_points[-3:] + all_points[0:-3:3]
    points_last = np.concatenate(points, axis=0)
    # robot.sleep(2.)
    # robot.setVelosities(.5, 0)
    # robot.sleep(2.)
    points_b = get_points(robot.getLaser(), robot.getDirection())
    # plt.plot(points_b[:, 0], points_b[:, 1])
    # plt.scatter(points_last[:, 0], points_last[:, 1])
    # points_c = np.dot(m, np.concatenate((points_b, np.ones((points_b.shape[0], 1))), axis=-1).copy().T).T

    # plt.plot(points_c[:, 0], points_c[:, 1])

    points_d, m_, err = align(points_last, points_b, m, keep_thresh=.65, iterations=59)
    # m_, err, points_d = np.identity(3), 0., points_b
    # points_de = np.concatenate((points_d, np.ones((points_d.shape[0], 1))), axis=-1).copy()
    # points_de = np.dot(m_, points_de.T).T[:, :2]
    m = np.dot(m, m_)
    origin = m[:2, 2]
    # print(origin)

    if plot:
        for point in all_points:
            plt.scatter(point[:, 0], point[:, 1])
        plt.plot(points_d[:, 0], points_d[:, 1], c="black")
        plt.scatter(origin[0], origin[1], c="gray", marker='x')#"cross")
        plt.show()

    # print(err)
    img_, tile_ = create_img(origin, points_d, 512, 5)
    img += img_
    tile += tile_
    if draw:
        cv2.imshow("img", maxnorm(img).astype(np.uint8))
        cv2.imshow("tile", maxnorm(tile).astype(np.uint8))
        # cv2.imshow("img", ((img > 1) * 255).astype(np.uint8))
        # robot.setVelosities(0, 0)
        cv2.waitKey(1)
    # all_points = [points_d]
    if key:
        all_points.append(points_d[np.all(np.isfinite(points_d), axis=-1), :2])
    # points_last = np.concatenate((points_last, points_d[np.all(np.isfinite(points_d), axis=-1), :2]))
    return err, origin


def maxnorm(img):
    return (img / img.max()) * 255


def bfs(world, costs, source, trarge):
    conv = lambda x: tuple(int(x) for x in (world.shape[0] * world2img(x)).flatten())
    start = conv(source)
    trarge = conv(trarge)
    # print(start, trarge)
    que = [start + ((), 0)]
    vis = {start}
    while True:
        que.sort(key=lambda x: x[-1])
        try:
            px, py, hist, cost = que.pop()
        except IndexError:
            return np.array([], dtype=np.int32)
        for x in range(-1, 1):
            for y in range(-1, 1):
                if x == y == 0:
                    continue
                point = (px+x, py+y)
                if point in vis:
                    continue
                if px == 0 or py == 0:
                    continue
                if world[point[1], point[0]] and not world[py, px]:
                    continue
                vis.add(point)
                point += (hist + (point,), cost + costs[point[1], point[0]])
                que.append(point)
                if world[py, px]:
                    if not world[point[1], point[0]]:
                        print('from')
                        que = [point]
                elif point[:2] == trarge:
                    # print(point)
                    return np.array(point[-2], dtype=np.int32)

def stb(x):
    return (x + np.pi * 5) % (np.pi * 2) - (np.pi)

def reg_forward(goal):
    _, start_position = rem(key=False)
    direction = goal - start_position
    turn_angle(np.arctan2(direction[1], direction[0]))
    compass = compass_error()

    position = start_position
    bs = np.sum((goal - start_position) ** 2)
    def error():
        return compass()
        nonlocal position
        _, position = rem(key=False)
        direction = goal - position
        target = np.arctan2(direction[1], direction[0])
        turn = target - robot.getDirection()
        return stb(turn) * 50
        for map in all_points:
            plt.scatter(map[:, 0], map[:, 1])
        plt.scatter(position[0], position[1], marker='x', c='gray')
        plt.show()
        return compass()

    encoder_start = robot.getEncoders()["left"]
    _, fs, _ = sensor()
    compass = compass_error()
    def termination():
        _, f, _ = sensor()
        encoder_current = robot.getEncoders()["left"]
        if f <= bs * 2:
            return (abs(fs - f) > bs) or (encoder_current - encoder_start >= m2cpr(bs + .1))
        return compass()
        # nonlocal position
        # _, position = rem(key=False)
        # return np.dot(position-start_position, goal-start_position) / np.linalg.norm(goal-start_position) > 1
        # encoder_current = robot.getEncoders()["left"]
        # _, f, _ = sensor()
        # return (encoder_current - encoder_start >= m2cpr(bs)) or (f < bff_limit)

    # PID
    pid(error, termination, vel=.3)


if __name__ == "__main__":

    # initialize robot
    robot = Robot()
    #
    # right()
    # left()
    # left()
    # right()
    # right()

    # help(cv2.ppf_match_3d)
    # exit()

    # robot.setVelosities(0., +.6)
    # robot.setVelosities(0., -.6)
    robot.sleep(0.65)
    # robot.sleep(1.85)
    robot.setVelosities(0., -0.)
    # robot.sleep(1.5)
    points_a = get_points(robot.getLaser(), robot.getDirection())
    offset = np.zeros(2)
    img, tile = create_img(offset, points_a, 512, 5)

    m = np.identity(3)
    points_last = points_a
    all_points = [points_last]
    # keyframe()
    # robot.setVelosities(0., -0.)
    # robot.sleep(1.5)
    # exit()
    for i in range(150):
        # print(i)
        if i == 0 or i:
            robot.setVelosities(.5, .0)
            robot.sleep(.5)
            robot.setVelosities(0, 0)
        _, offset = rem(plot=True)#, plot=True)
        # print(offset)
        if i < -1:
            obst = (maxnorm(img) > .1).astype(np.uint8)
            off = 1
            obst = cv2.dilate(obst, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (off, off)))
            dmp = cv2.distanceTransform(1-obst, cv2.DIST_L1, 5)
            # dmp[dmp < 15] -= 500000
            # dmp = np.minimum(dmp, 455555.) * 150
            is_voronoi = dmp == np.maximum(dmp, np.maximum(np.roll(dmp, 1, axis=0), np.roll(dmp, -1, axis=0)))
            dmp *= 100.
            dmp = cv2.ximgproc.thinning((1-obst) * 255)

            inds = np.stack(np.mgrid[0:512, 0:512], axis=-1)
            possible = inds[dmp > 0].reshape((-1, 2))
            possible = (possible / 512. - 0.5) * 10.
            goal = np.array((-2., -2.))
            goal = possible[np.argmin(np.sum((possible - goal) ** 2))]

            hist = bfs(dmp #  obst
                       , np.zeros_like(
                        is_voronoi
                       ),
                       offset, np.array((-2., -2.)))
            obst[hist[:, 1], hist[:, 0]] = 1
            # waypoint = np.array((-2., -2.))
            # waypoint = (hist[1] / img.shape[0] - 0.5) * 10.
            # origin = waypoint - offset
            origin = ((hist[1] - hist[0]) / img.shape[0] - 0.5) * 10.
            angle = np.arctan2(origin[1], origin[0])
            if angle < 0:
                angle = 2 * np.pi + angle
            # turn_angle(angle)
            # forward(bs=.2)
            # reg_forward(waypoint)
            # forward(bs=min(.3, np.sum(origin ** 2)), controller=0)
            # err = stb(std(angle) - std(robot.getDirection()))
            # err *= 4.
            robot.setVelosities(0, 0)
            # err = np.sign(err) * min(.5, abs(err))
            # print(err)
            # robot.setVelosities(.2, -err)
            cv2.imshow("obst", obst * 255)
            cv2.imshow("dmp", dmp / dmp.max())
            cv2.imshow("vor", is_voronoi.astype(np.uint8) * 255)
            cv2.waitKey(5000)
            # plt.imshow(obst)
            # plt.plot(hist[:, 0], hist[:, 1])
            # plt.show()
            # print(hist)
            # print(hist.shape)
            # origin = hist[1] - hist[0]
            # print(origin)
            # angle = np.arctan2(origin[1], origin[0])
            # direction = robot.getDirection()
            # turns = -(std(angle) - std(direction))
            # turns = np.sign(turns) * min(.4, abs(turns))
            # if abs(turns) > np.pi:
            #     turns = (abs(turns) - np.pi) * -np.sign(turns)

            # print(turns)
            # robot.setVelosities(.2, turns * 1)
            # pf = np.zeros_like(img, dtype=np.float32)
            # pf -= cv2.GaussianBlur(maxnorm(img), (23, 23), 7)
            # plt.imshow(pf)
            # plt.show()
        robot.sleep(0.05)
    cv2.waitKey(0)
    exit()
    plt.axes().set_aspect('equal')
    plt.plot(points_a[:, 0], points_a[:, 1])
    plt.plot(points_b[:, 0], points_b[:, 1])
    # for points_d in ds[-1:]:

        # points_d = np.concatenate((points_b, np.ones((points_b.shape[0], 1))), axis=-1).copy()
        # points_d = np.dot(m, points_d.T).T
    plt.plot(points_d[:, 0], points_d[:, 1])
    plt.show()
    exit()


    img_1 = create_img(points_a)
    img_2 = create_img(points_b)
    tx,ty = 0, 0
    for i in range(15):
        print(i)
        costs = []
        for x in range(-1, 1):
            for y in range(-1, 1):
                img_3 = trans(img_1, x, y)
                costs.append((np.sum(img_3 * img_2), (x, y), img_3))
        _, (x, y), img_1 = max(costs)
        tx, ty = tx + x, ty + y
    cv2.imshow("hey", img_1 * 127 + img_2 * 127)
    cv2.waitKey(0)
    exit()
    cv2.waitKey(0)

    plt.axes().set_aspect('equal')
    plt.plot(points_a[:, 0], points_a[:, 1])
    plt.plot(points_b[:, 0], points_b[:, 1])
    plt.show()
    exit()

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

    arr = """
    [0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0]
    [1 1 1 0 0 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 0 0 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 0 0 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 0 0 1 1 1 0 0 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]
    [0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0]"""[1:]
    zs = []
    for i in arr.splitlines(keepends=False):
        xs = [int(x) for x in i[1:-1].split()]
        zs.append(xs)
    zs = np.array(zs)
    zs[0] = 1
    zs[-1] = 1
    zs[:, 0] = 1
    zs[:, -1] = 1
    blocks = []
    print(zs)
    for y in range(zs.shape[1]):
        for x in range(zs.shape[0]):
            if zs[y, x] == 0:
                zs[y:y + 2, x:x + 2] = 1
                blocks.append((x, y))
    min_dist = [min(block[0], block[1], 16 - block[0] - 2, 16 - block[1] - 2) for block in blocks]
    dist, block = min(zip(min_dist, blocks))

    exit()
