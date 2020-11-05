import datetime
import random

from .util import *

#save info
def Save_info(save_path, path, Size, Goal_num, robot_list, t):
    if save_path:
        now_time = datetime.datetime.now()
        tmp = "Robot " + str(Size) + " Target " + str(Goal_num) + " Iteration " + str(t)
        p = path + tmp + datetime.datetime.strftime(now_time, '%Y-%m-%d %H:%M:%S')
        with open(p, 'w') as wf:

            for i in range(len(robot_list)):
                x_l = robot_list[i].list_x
                y_l = robot_list[i].list_y
                wf.write('robot ')
                wf.write(str(i))
                wf.write('\n')
                for j in range(len(x_l)):
                    wf.write('(')
                    wf.write(str(x_l[j]))
                    wf.write(',')
                    wf.write(str(y_l[j]))
                    wf.write(')')

                wf.write('\n')

# update Gbest Pbest
def update_GbestPbest(r, Gx, Gy, Gbest, goal_list_x, goal_list_y, robot_list, v_x, v_y, i, t):
    gx = Gx
    gy = Gy
    gbest = Gbest
    f = 0
    for ttt in range(len(goal_list_x)):
        tmp_f = getFitness(goal_list_x[ttt], goal_list_y[ttt], robot_list[i].x, robot_list[i].y)
        f += tmp_f

    if f <= r.pbest:
        r.pbest_p = r.pbest
        r.pbest = f
        r.pbest_avg = (r.pbest_avg * t + f) / (t + 1)
        r.px = r.x
        r.py = r.y
    if f <= gbest:
        gbest = f
        gx = r.px
        gy = r.py
    if can_print == 1:
        print("position of {} is {}+{}={} {}+{}={}  length = {}".format(i, int(r.x - v_x), v_x, int(r.x),
                                                                        int(r.y - v_y), v_y, int(r.y),
                                                                        math.sqrt(v_x * v_x + v_y * v_y)))
        print("position of {} ".format(i))
        print("f = {}".format(f))
        print(goal_list_x)
        print(goal_list_y)
    return gbest, gx, gy

# limit max velocity
def Limit_maxVelocity(v_x, v_y, v_limit):
    if abs(v_x) > v_limit / 2 or abs(v_y) > v_limit / 2:
        k = v_x * v_x + v_y * v_y
        k = math.sqrt(k)
        v_x = v_x * v_limit / k
        v_y = v_y * v_limit / k
    return v_x, v_y

def Unit_speed(r, gx, gy):
    xp = r.px - r.x
    xg = gx - r.x
    yp = r.py - r.y
    yg = gy - r.y
    p_len = math.sqrt(xp ** 2 + yp ** 2)
    if p_len > 0:
        xp = xp / p_len
        yp = yp / p_len
    g_len = math.sqrt(xg ** 2 + yg ** 2)
    if g_len > 0:
        xg = xg / g_len
        yg = yg / g_len
    return xp, yp, xg, yg

# Avoid obstacle
def avoidObstacle(r, obs_list , robot_list):
    po_x = r.x
    po_y = r.y
    ob = 0
    for jj in range(len(obs_list)):
        if detect_ob(r.x, r.y, obs_list[jj]) == 1:
            ob = 1
            break

    for jj in range(len(robot_list)):
        if detect_ob_self(r.x, r.y, robot_list[jj]) == 1:
            ob = 1
            break

    # x,y direction
    zf_index_x = 1
    zf_index_y = 1
    if r.Vx != 0:
        zf_index_x = r.Vx / abs(r.Vx)
    if r.Vy != 0:
        zf_index_y = r.Vy / abs(r.Vy)

    # 8 direction
    tmp_list1 = [3, 3, 0, 3, -3, 0, -3, -3]
    tmp_list2 = [3, 0, 3, -3, 3, -3, 0, -3]

    # find 1 best direction in 8 to avoid obstacle
    if ob == 1:
        c3 = 100000
        for ixx in range(len(tmp_list1)):
            ii = tmp_list1[ixx]
            ij = tmp_list2[ixx]

            ii = ii * zf_index_x
            ij = ij * zf_index_y

            ob11 = 0
            for jj in range(len(obs_list)):

                if detect_ob(r.x + ii, r.y + ij, obs_list[jj]) == 1:
                    ob11 = 1
                    break

            for jj in range(len(robot_list)):
                if detect_ob_self(r.x + ii, r.y + ij, robot_list[jj]) == 1:
                    ob11 = 1
                    # print("ob with robots")
                    break
            if (ob11 == 0):
                po_x += ii
                po_y += ij
                break

        if can_print == 1:
            print("ob {} {}".format(po_x, po_y))

    return ob, po_x, po_y


# Find the farthest & emptiest area
def find_FarthestAndEmptiestArea(r, Out_list , SIDE):
    # Get current robot's position area
    area_id = int(int((r.y - 1) / 50) * SIDE / 50 + int(r.x / 50))
    far_area_id = 0
    tmp_num = -10000

    # Find the farthest & emptiest area
    for j in range(len(Out_list)):
        x_d = abs(area_id % 20 - j % 20)
        y_d = int(abs(area_id - j) / 20)
        dis = math.sqrt(x_d * x_d + y_d * y_d)

        if (Out_list[j] * dis > tmp_num):
            tmp_num = Out_list[j] * dis
            far_area_id = j
    return area_id, far_area_id


def judge_allGoalReached(goal_list_x, goal_list_y, robot_list ,goal_c, i , Gbest):
    gbest = Gbest
    for xi in range(len(goal_list_x)):
        if getFitness(goal_list_x[xi], goal_list_y[xi], robot_list[i].x, robot_list[i].y) < 2:
            goal_c[xi] = goal_c[xi] + 1
    for xi in range(len(goal_c)):
        if (goal_c[xi] >= 1 and len(goal_c) > 0 and xi < len(goal_c)):
            del goal_list_x[xi]
            del goal_list_y[xi]
            del goal_c[xi]

            gbest = 100000
            break
    return goal_list_x , goal_list_y , goal_c , gbest

def judge_robotReachGoal(goal_list_x, goal_list_y, robot_list,i):
    reach = 0
    for xi in range(len(goal_list_x)):
        if getFitness(goal_list_x[xi], goal_list_y[xi], robot_list[i].x, robot_list[i].y) < 2:
            reach = 1
    return reach




def update_best_avg(robot_list):
    best_avg = 0
    for i in range(len(robot_list)):
        r = robot_list[i]
        best_avg = best_avg + r.pbest
    best_avg = best_avg / len(robot_list)
    return best_avg

def init_E2RPSO(SIDE,Size,Goal_num):
    # save data
    Out_list = []
    robot_list = []
    obs_list = []

    for i in range(int(int(200) * int(200))):
        Out_list.append(100)
    # init Map area exploration rate
    for i in range(int(int(SIDE / 20) * int(SIDE / 20))):
        Out_list.append(100)

    # init goal list (x position)
    goal_list_x = [450 / 1000 * SIDE, 700 / 1000 * SIDE, 500 / 1000 * SIDE, 200 / 1000 * SIDE, 600 / 1000 * SIDE,
                   200 / 1000 * SIDE, 800 / 1000 * SIDE, 800 / 1000 * SIDE, 400 / 1000 * SIDE, 300 / 1000 * SIDE]
    # init goal list (y position)
    goal_list_y = [500 / 1000 * SIDE, 450 / 1000 * SIDE, 200 / 1000 * SIDE, 500 / 1000 * SIDE, 650 / 1000 * SIDE,
                   200 / 1000 * SIDE, 200 / 1000 * SIDE, 700 / 1000 * SIDE, 400 / 1000 * SIDE, 800 / 1000 * SIDE]
    # Auxiliary judgment abandons the target
    goal_c = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # goal num
    goal_list_x = goal_list_x[0:Goal_num]
    goal_list_y = goal_list_y[0:Goal_num]
    goal_c = goal_c[0:Goal_num]

    # init robots
    size = Size
    for i in range(size):
        x1 = (i % 4 * 20) / 1000 * SIDE
        y1 = (400 + i / 4 * 20) / 1000 * SIDE

        robot_list.append((Robot(i, random.uniform(-5, 5), random.uniform(-5, 5), x1, y1)))

    # init obstancle

    obs_list.append(Obstacle(1, 90 / 1000 * SIDE, 540 / 1000 * SIDE, 30 / 1000 * SIDE, 80 / 1000 * SIDE))
    obs_list.append(Obstacle(2, 360 / 1000 * SIDE, 270 / 1000 * SIDE, 65 / 1000 * SIDE, 80 / 1000 * SIDE))
    obs_list.append(Obstacle(3, 540 / 1000 * SIDE, 720 / 1000 * SIDE, 140 / 1000 * SIDE, 50 / 1000 * SIDE))
    obs_list.append(Obstacle(4, 270 / 1000 * SIDE, 720 / 1000 * SIDE, 83 / 1000 * SIDE, 30 / 1000 * SIDE))
    obs_list.append(Obstacle(5, 589 / 1000 * SIDE, 600 / 1000 * SIDE, 40 / 1000 * SIDE, 185 / 1000 * SIDE))
    obs_list.append(Obstacle(6, 300 / 1000 * SIDE, 550 / 1000 * SIDE, 45 / 1000 * SIDE, 150 / 1000 * SIDE))

    # init gx,gy,gbest
    gbest = GBEST
    gx = 0
    gy = 0
    t = 0.0  # iteration time

    # init Gbest Pbest
    for i in range(len(robot_list)):
        f = 0
        for ttt in range(len(goal_list_x)):
            tmp_f = getFitness(goal_list_x[ttt], goal_list_y[ttt], robot_list[i].x, robot_list[i].y)
            f += tmp_f

        if f < robot_list[i].pbest:
            robot_list[i].pbest_p = f
            robot_list[i].pbest = f
            robot_list[i].pbest_avg = f
            robot_list[i].px = robot_list[i].x
            robot_list[i].py = robot_list[i].y
            if f < gbest:
                gbest = f
                gx = robot_list[i].px
                gy = robot_list[i].py
        if can_print == 1:
            print("position of {} with fitness {} is {} {}".format(i, robot_list[i].pbest, robot_list[i].x,
                                                                   robot_list[i].y))

    return Out_list,robot_list,obs_list,goal_list_x,goal_list_y,goal_c

#Draw
# def Draw(SIDE, obs_list, robot_list, ani_list , t):
#     if config.getboolean('E2RPSO' , 'show_animation'):
#         if config.getint('E2RPSO','show_type') == 1:
#             fig = plt.figure()
#             ax = fig.add_subplot(111)
#
#             plt.plot(450 / 1000 * SIDE, 500 / 1000 * SIDE, "xb")
#             plt.plot(700 / 1000 * SIDE, 450 / 1000 * SIDE, "xb")
#             plt.plot(500 / 1000 * SIDE, 200 / 1000 * SIDE, "xb")
#             plt.plot(200 / 1000 * SIDE, 500 / 1000 * SIDE, "xb")
#             plt.plot(600 / 1000 * SIDE, 650 / 1000 * SIDE, "xb")
#             plt.plot(200 / 1000 * SIDE, 200 / 1000 * SIDE, "xb")
#             plt.plot(800 / 1000 * SIDE, 200 / 1000 * SIDE, "xb")
#             plt.plot(800 / 1000 * SIDE, 700 / 1000 * SIDE, "xb")
#             plt.plot(400 / 1000 * SIDE, 400 / 1000 * SIDE, "xb")
#             plt.plot(300 / 1000 * SIDE, 800 / 1000 * SIDE, "xb")
#
#             plt.plot(0 / 1000 * SIDE, 1000 / 1000 * SIDE, "x")
#             plt.plot(0, 0, "x")
#             plt.plot(1000 / 1000 * SIDE, 1000 / 1000 * SIDE, "x")
#             plt.plot(1000 / 1000 * SIDE, 0, "x")
#
#             for i in range(len(obs_list)):
#                 rect = plt.Rectangle((obs_list[i].x, obs_list[i].y - obs_list[i].width), obs_list[i].length,
#                                      obs_list[i].width)
#                 ax.add_patch(rect)
#
#             plt.plot()
#             for i in range(len(robot_list)):
#                 plt.plot(robot_list[i].list_x, robot_list[i].list_y, label="ss")
#
#             plt.pause(0.0001)
#
#         if config.getint('E2RPSO','show_type') == 2:
#
#             fig = plt.figure(tight_layout=True)
#             ax = fig.add_subplot(111)
#             for j in range(len(obs_list)):
#                 rect = plt.Rectangle((obs_list[j].x, obs_list[j].y - obs_list[j].width), obs_list[j].length,
#                                      obs_list[j].width)
#                 ax.add_patch(rect)
#
#             plt.plot(0, 1000 / 1000 * SIDE, "x")
#             plt.plot(0, 0, "x")
#             plt.plot(1000 / 1000 * SIDE, 1000 / 1000 * SIDE, "x")
#             plt.plot(1000 / 1000 * SIDE, 0, "x")
#
#             plt.plot(450 / 1000 * SIDE, 500 / 1000 * SIDE, "xb")
#             plt.plot(700 / 1000 * SIDE, 450 / 1000 * SIDE, "xb")
#             plt.plot(500 / 1000 * SIDE, 200 / 1000 * SIDE, "xb")
#             plt.plot(200 / 1000 * SIDE, 500 / 1000 * SIDE, "xb")
#             plt.plot(600 / 1000 * SIDE, 650 / 1000 * SIDE, "xb")
#             plt.plot(200 / 1000 * SIDE, 200 / 1000 * SIDE, "xb")
#             plt.plot(800 / 1000 * SIDE, 200 / 1000 * SIDE, "xb")
#             plt.plot(800 / 1000 * SIDE, 700 / 1000 * SIDE, "xb")
#             plt.plot(400 / 1000 * SIDE, 400 / 1000 * SIDE, "xb")
#             plt.plot(300 / 1000 * SIDE, 800 / 1000 * SIDE, "xb")
#
#             for i in range(len(robot_list)):
#                 tmp, = plt.plot(robot_list[i].x, robot_list[i].y, "ro", markersize=.5)
#                 ani_list.append(tmp)
#
#             ani = animation.FuncAnimation(fig, update_points, np.arange(0, t), interval=5, blit=True)
#             plt.show()
#
# # function for Draw dynamic
# def update_points(num,ani_list,robot_list):
#     ht = int(num)
#     for i in range(len(ani_list)):
#         if ht < len(robot_list[i].list_x):
#             ani_list[i].set_data(int(robot_list[i].list_x[ht]), int(robot_list[i].list_y[ht]))
#     return ani_list