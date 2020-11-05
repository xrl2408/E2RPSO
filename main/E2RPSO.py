import math
from enum import Enum

import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
import matplotlib.animation as animation
import datetime
import numpy as np
import time
import random
import configparser
from util import E2RPSO_util

config = configparser.ConfigParser()

config_path = r'configuration/setting.ini'

config.read(config_path)

PBEST = config.getint('E2RPSO','pbest')
GBEST = config.getint('E2RPSO','gbest')
save = config.getint('E2RPSO','save')
can_print = config.getint('E2RPSO','can_print')
Path = "/Users/kumazuirin/Desktop/"
V_LIMIT = config.getint('E2RPSO','v_limit')

# save data
ani_list = []  # trajectory info
Out_list = []
robot_list = []
obs_list = []
for i in range(int(int(200) * int(200))):
    Out_list.append(100)


'''
function E2RPSO
    main function of PSO

    parameter:
        Side        : the map size
        Size        : num of robots
        Goal_num    : num of goal

    returns
        Iteration time, goal num found


'''


def E2RPSO():
    # Side length of square map
    SIDE = config.getint('E2RPSO','SIDE')
    Size = config.getint('E2RPSO','Size')
    Goal_num = config.getint('E2RPSO','Goal_num')

    # Max iteration times
    T = config.getint('E2RPSO','T')

    # PSO parameters
    C1 = config.getfloat('E2RPSO','C1')
    C2 = config.getfloat('E2RPSO','C2')
    C3_index = config.getint('E2RPSO','C3_index')
    C4_index = config.getint('E2RPSO','C4_index')
    K = config.getint('E2RPSO','K')

    C3 = C3_index * C2
    C4 = C4_index * SIDE

    gbest = GBEST
    gx = 0
    gy = 0
    t = 0.0  # iteration time

    # initial
    Out_list, robot_list, obs_list, goal_list_x, goal_list_y, goal_c = E2RPSO_util.init_E2RPSO(SIDE,Size,Goal_num)

    c1 = C1
    c2 = C2


    # begin iteration
    while t < T:
        t = t + 1

        # update best_avg
        best_avg = E2RPSO_util.update_best_avg(robot_list)

        # print info
        if can_print == 1:
            print("---------------{}---------------".format(t))
            print("g_best = {} {}".format(gx, gy))
            print("g_best = {}".format(gbest))

        # if find all target , return
        if len(goal_list_x) == 0:
            break

        # Traverse all robots
        for i in range(len(robot_list)):
            # do avoid local best every 10 iteration
            if t % 10 == 0:
                c1 = C1
                c2 = C2
            c3 = 0
            r = robot_list[i]
            if r.reach == 1:
                continue
            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
            r3 = random.uniform(0, 1)

            # this robot already reach one goal
            if r.reach == 1:
                continue

            # end?
            if len(goal_list_x) == 0:
                break

            # update inertial component
            w = 0.9 - 0.5 * (1 - (min(r.pbest_p, r.pbest) / max(r.pbest_p, r.pbest))) + (
                        min(gbest, best_avg) / max(gbest, best_avg))

            # Judge whether the current robot has reached the goal point
            reach = E2RPSO_util.judge_robotReachGoal(goal_list_x, goal_list_y, robot_list,i)

            # Judge whether the goal is reached and update info
            goal_list_x, goal_list_y, goal_c, gbest = E2RPSO_util.judge_allGoalReached(goal_list_x, goal_list_y, robot_list ,goal_c, i , gbest)

            # if find all target , return
            if len(goal_c) == 0:
                break

            # if this robot reach a goal, stop
            if (reach == 1):
                # print("reach of robot {}".format(i))
                robot_list[i].reach = 1
                continue

            # Find the farthest & emptiest area
            area_id, far_area_id = E2RPSO_util.find_FarthestAndEmptiestArea(r, Out_list , SIDE)


            # farthest & emptiest exploration rate increment - K
            Out_list[far_area_id] = Out_list[far_area_id] - K

            # Determine whether to enter local optimum or not, then set C3
            if Out_list[area_id] < -800:
                c2 = 0
                c1 = 0
                c3 = 10 * C3
            elif Out_list[area_id] < -400:
                c3 = C3
                c2 = 0
                c1 = 0

            # prepare for unit speed x,y component
            pu_x = ((far_area_id + 1) % 20) * 50
            pu_y = int((far_area_id) / 20 + 1) * 50

            # avoid obstacle and decide the avoiding obstacle point
            ob, po_x, po_y = E2RPSO_util.avoidObstacle(r, obs_list , robot_list)
            if ob == 1:
                pa_x = po_x
                pa_y = po_y
            else:
                pa_x = pu_x
                pa_y = pu_y

            # Unit speed for x and y component
            xp, yp, xg, yg = E2RPSO_util.Unit_speed(r, gx, gy)

            # update velocity , using E2RPSO
            v_x = w * r.Vx + r1 * c1 * xp + c2 * r2 * xg + c3 * r3 * (pa_x - r.x)
            v_y = w * r.Vy + r1 * c1 * yp + c2 * r2 * yg + c3 * r3 * (pa_y - r.y)

            # limit max velocity
            v_x, v_y = E2RPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)
            r.Vx = v_x
            r.Vy = v_y

            # Auxiliary judgment enters local optimum
            if t % 10 == 0:
                r.lastx = r.x + v_x
                r.lasty = r.y + v_y

            # update position
            r.x = r.x + v_x
            r.y = r.y + v_y

            # deal root out of map
            if (r.x < 0 or r.y < 0 or r.x > SIDE or r.y > SIDE):
                r.x = r.x - v_x
                r.y = r.y - v_y
                r.Vx = -r.Vx
                r.Vy = -r.Vy

            # Auxiliary draw
            r.list_x.append(r.x)
            r.list_y.append(r.y)

            # update Pbest & Gbest
            gbest, gx, gy = E2RPSO_util.update_GbestPbest(r, gx, gy, gbest, goal_list_x, goal_list_y, robot_list, v_x, v_y, i, t)
            Out_list[int(int((r.y - 1) / 50) * SIDE / 50 + int(r.x / 50))] = Out_list[int(
                int((r.y - 1) / 50) * SIDE / 50 + int(r.x / 50))] - 1


    # save path info
    E2RPSO_util.Save_info(config.getboolean('E2RPSO' , 'save_path'), config.get('E2RPSO' , 'path'), Size, Goal_num, robot_list, t)

    return int(t), Goal_num - len(goal_list_x)


# function for Draw dynamic
def update_points(num):
    ht = int(num)
    for i in range(len(ani_list)):
        if ht < len(robot_list[i].list_x):
            ani_list[i].set_data(int(robot_list[i].list_x[ht]), int(robot_list[i].list_y[ht]))
    return ani_list



result = E2RPSO()




