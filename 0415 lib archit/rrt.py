import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from lib.calculateFK import FK


def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []

    # get joint limits
    lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    Ts = [start]  # create a set for T start
    Tg = [goal]  # create a set for T goal
    # print(Ts)
    # print(Tg)
    Ts_r = []  # create an empty set
    Tg_r = []
    # print(map)
    # result_s = np.any(detectCollision(linePt1_s,linePt2_s,map))
    # result_s = isRobotCollided(start,map)
    # result = np.any([False,False,True])
    # print(result_s)
    # result_g = isRobotCollided(goal,map)
    # print(result_g)

    step_number = 0
    path_f = []
    path_l = []
    # '''
    while True:

        if isRobotCollided(start, map) or isRobotCollided(goal, map):
            break

        q0 = (np.random.randint(-28973, 28973)) / 10000
        q1 = (np.random.randint(-17628, 17628)) / 10000
        q2 = (np.random.randint(-28973, 28973)) / 10000
        q3 = (np.random.randint(-30718, -698)) / 10000
        q4 = (np.random.randint(-28973, 28973)) / 10000
        q5 = (np.random.randint(-175, 37525)) / 10000
        q6 = (np.random.randint(-28973, 28973)) / 10000

        q = np.array([q0, q1, q2, q3, q4, q5, q6])  # find a random q in configuration space

        while isRobotCollided(q, map):
            q0 = (np.random.randint(-28973, 28973)) / 10000
            q1 = (np.random.randint(-17628, 17628)) / 10000
            q2 = (np.random.randint(-28973, 28973)) / 10000
            q3 = (np.random.randint(-30718, -698)) / 10000
            q4 = (np.random.randint(-28973, 28973)) / 10000
            q5 = (np.random.randint(-175, 37525)) / 10000
            q6 = (np.random.randint(-28973, 28973)) / 10000

            q = np.array([q0, q1, q2, q3, q4, q5, q6])  # find a random q in configuration space

        norm = []

        for i in range(0, len(Ts)):  # find the number of closest node
            norm = np.concatenate((norm, [np.linalg.norm(q - Ts[i])]), axis=0)
            if norm[i] == np.sort(norm)[0]:
                min = i

        qa = Ts[min]
        max_distance = np.max(np.absolute(qa - q))
        steps = max_distance / 0.05  # 0.1 rads (6 deg) per step
        steps = int(steps)
        q_step = (q - qa) / steps

        m = 1
        for i in range(0, steps):
            q_new = qa + i * q_step
            if isRobotCollided(q_new, map):
                m = 0
                break

        if m == 1:
            Ts.append(q)
            Ts_r.append(qa)

        norm = []

        for i in range(0, len(Tg)):  # find the number of closest node
            norm = np.concatenate((norm, [np.linalg.norm(q - Tg[i])]), axis=0)
            if norm[i] == np.sort(norm)[0]:
                min = i

        qb = Tg[min]
        max_distance = np.max(np.absolute(qb - q))
        steps = max_distance / 0.1  # 0.1 rads (6 deg) per step
        steps = int(steps)
        q_step = (q - qb) / steps

        n = 1
        for i in range(0, steps):
            q_new = qb + i * q_step
            if isRobotCollided(q_new, map):
                n = 0
                break

        if n == 1:
            Tg.append(q)
            Tg_r.append(qb)

        step_number = step_number + 1

        x = 1

        if step_number > 500:

            break

        elif (Tg[-1] == Ts[-1]).all():
            x = 2
            path_f.append(Ts[-1])
            m = len(Ts)
            while (path_f[-1] != Ts[0]).all():
                for i in range(0, m - 1):
                    if (Ts[i] == Ts_r[m - 2]).all():
                        path_f.append(Ts[i])
                        m = i + 1
                        break

            path_f.reverse()

            path_l.append(Tg[-1])
            n = len(Tg)
            while (path_l[-1] != Tg[0]).all():
                for i in range(0, n - 1):
                    if (Tg[i] == Tg_r[n - 2]).all():
                        path_l.append(Tg[i])
                        n = i + 1
                        break

            path_l = path_l[1:len(path_l)]

            path_f.extend(path_l)

            path = path_f

            break

    # '''
    # print(Ts[-1])
    # print(goal)
    # print(path[-1])
    # Ts.append(goal)
    # print(Ts)
    # T = Ts - Ts_r
    # print(T)
    # Ts_r.append(goal)
    # print(Ts_r)
    # Ts.reverse()
    # print(Ts)

    # norm = np.sort(np.concatenate((start,[2]),axis=0))[0]
    # print(norm)

    # Ts_r = Ts_r.append(start)
    # Ts_r.append(start)
    # Ts_r.append(goal)
    # Ts_r = Ts_r[0]
    # T0 = T[0]
    # T1 = T[1]
    # print(Ts_r)
    # print(np.shape(Ts_r))
    # print(Ts)
    # print(T0)
    print(step_number)
    # print(path_l)
    # print(path_f)
    # print(path_l)
    print(path)
    # print(Tg)
    # print(Ts)
    # print(x)
    # print(Tg[len(Tg)-1].all() == Ts[len(Ts)-1].all())
    return path


def isRobotCollided(q, map):
    # result = 0
    JP = FK().forward(q)[0]
    linePt1 = JP[1:8, :]
    # print(np.shape(linePt1))
    linePt2 = JP[0:7, :]
    # print(linePt2)
    # print(np.shape(map[0]))
    # print(map)
    result = False
    for i in range(0, len(map[0])):
        # result = np.any(detectCollision(linePt1,linePt2,map[0][i]))
        # print(result)
        # print(map[0][i])
        if np.any(detectCollision(linePt1, linePt2, map[0][i])):
            result = True
            break
        else:
            result = False

    return result


if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0, -1, 0, -2, 0, 1.57, 0])
    goal = np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7])
    # map = map_struct[0]
    # print(len(map))
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))


# # test code of is robot collided
# if __name__ == '__main__':
#
#     map_struct = loadmap("../maps/map2.txt")
#
#     q0 = (np.random.randint(-28973,28973))/10000
#     q1 = (np.random.randint(-17628,17628))/10000
#     q2 = (np.random.randint(-28973,28973))/10000
#     q3 = (np.random.randint(-30718,-698))/10000
#     q4 = (np.random.randint(-28973,28973))/10000
#     q5 = (np.random.randint(-175,37525))/10000
#     q6 = (np.random.randint(-28973,28973))/10000
#
#     q = np.array([q0,q1,q2,q3,q4,q5,q6])
#
#     result = isRobotCollided(q,map_struct)
#
#     print(map_struct)
#     print(q)
#     print(result)
