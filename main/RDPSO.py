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
from util import RDPSO_util

config = configparser.ConfigParser()

config_path = r'configuration/setting.ini'

config.read(config_path)

PBEST = config.getint('RDPSO','pbest')
GBEST = config.getint('RDPSO','gbest')
save = config.getint('RDPSO','save')
can_print = config.getint('RDPSO','can_print')
Path = "/Users/kumazuirin/Desktop/"
V_LIMIT = config.getint('RDPSO','v_limit')

# save data
ani_list = []  # trajectory info
Out_list = []
robot_list = []
obs_list = []
for i in range(int(int(200) * int(200))):
    Out_list.append(100)


'''
function RDPSO
    main function of PSO

    parameter:
        Side        : the map size
        Size        : num of robots
        Goal_num    : num of goal

    returns
        Iteration time, goal num found


'''


def RDPSO():
    # Side length of square map
    SIDE = config.getint('RDPSO','SIDE')
    Size = config.getint('RDPSO','Size')
    Goal_num = config.getint('RDPSO','Goal_num')

    # Max iteration times
    T = config.getint('RDPSO','T')

    # PSO parameters
    C1 = config.getfloat('RDPSO','C1')
    C2 = config.getfloat('RDPSO','C2')


    gbest = GBEST
    gx = 0
    gy = 0
    t = 0.0  # iteration time

    # initial
    Out_list, robot_list, obs_list, goal_list_x, goal_list_y, goal_c = RDPSO_util.init_RDPSO(SIDE,Size,Goal_num)

    c1 = C1
    c2 = C2


    # begin iteration
    while t < T:
        t = t + 1

        w = config.getfloat('RDPSO', 'W_UPPERBOUND') - (t / T) * (
                    config.getfloat('RDPSO', 'W_UPPERBOUND') - config.getfloat('RDPSO', 'W_LOWERBOUND'))

        # update best_avg
        best_avg = RDPSO_util.update_best_avg(robot_list)

        # print info
        if can_print == 1:
            print("---------------{}---------------".format(t))
            print("g_best = {} {}".format(gx, gy))
            print("g_best = {}".format(gbest))

        # if find all target , return
        if len(goal_list_x) == 0:
            break

        # find and change fugitive
        if t % 50 == 0 and t > 0:
            robot_list.sort(key=lambda x: x.pbest)
            for i in range(0, int(20 / 5)):
                robot_list[20 - i - 1].fugitive = 1
                robot_list[i].fugitive = 0

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

            # Judge whether the current robot has reached the goal point
            reach = RDPSO_util.judge_robotReachGoal(goal_list_x, goal_list_y, robot_list,i)

            # Judge whether the goal is reached and update info
            goal_list_x, goal_list_y, goal_c, gbest = RDPSO_util.judge_allGoalReached(goal_list_x, goal_list_y, robot_list ,goal_c, i , gbest)

            # if find all target , return
            if len(goal_c) == 0:
                break

            # if this robot reach a goal, stop
            if (reach == 1):
                # print("reach of robot {}".format(i))
                robot_list[i].reach = 1
                continue

            # avoid obstacle and decide the avoiding obstacle point
            ob, po_x, po_y = RDPSO_util.avoidObstacle(r, obs_list , robot_list)
            pa_x = r.x
            pa_y = r.y
            if ob == 1:
                pa_x = po_x
                pa_y = po_y

            # Unit speed for x and y component
            xp, yp, xg, yg = RDPSO_util.Unit_speed(r, gx, gy)

            # update velocity , using RDPSO
            if r.fugitive == 0:
                v_x = w * r.Vx + r1 * c1 * xp + c2 * r2 * xg + c3 * r3 * (po_x - r.x)
                v_y = w * r.Vy + r1 * c1 * yp + c2 * r2 * yg + c3 * r3 * (po_y - r.y)

            else :
                v_x = (1+random.uniform(-1, 1))*w * vx + c3 * r3 * (po_x - r.x)
                v_y = (1+random.uniform(-1, 1))*w * vy + c3 * r3 * (po_y - r.y)

            # limit max velocity
            v_x, v_y = RDPSO_util.Limit_maxVelocity(v_x, v_y, V_LIMIT)
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
            gbest, gx, gy = RDPSO_util.update_GbestPbest(r, gx, gy, gbest, goal_list_x, goal_list_y, robot_list, v_x, v_y, i, t)
            Out_list[int(int((r.y - 1) / 50) * SIDE / 50 + int(r.x / 50))] = Out_list[int(
                int((r.y - 1) / 50) * SIDE / 50 + int(r.x / 50))] - 1


    # save path info
    RDPSO_util.Save_info(config.getboolean('RDPSO' , 'save_path'), config.get('RDPSO' , 'path'), Size, Goal_num, robot_list, t)

    return int(t), Goal_num - len(goal_list_x)


# function for Draw dynamic
def update_points(num):
    ht = int(num)
    for i in range(len(ani_list)):
        if ht < len(robot_list[i].list_x):
            ani_list[i].set_data(int(robot_list[i].list_x[ht]), int(robot_list[i].list_y[ht]))
    return ani_list



result = RDPSO()




