import math

import configparser

config = configparser.ConfigParser()  # 类实例化

config_path = r'configuration/setting.ini'

config.read(config_path)

PBEST = config.getint('E2RPSO','pbest')
GBEST = config.getint('E2RPSO','gbest')
save = config.getint('E2RPSO','save')
can_print = config.getint('E2RPSO','can_print')


'''
Robot class
    pbest_p     : Pbest in last iteration
    pbest       : Pbest in this iteration
    pbest_avg   : average Pbest
    x,y         : x , y value
    px,px       : pbest x,y value
    reach       : whether this robot reach goal
    list_x      : Store track information of x_position
    list_t      : Store track information of y_position
    lastx       : x value in last iteration
    lasty       : y value in last iteration
'''
class Robot:
    pbest_p = PBEST
    pbest = PBEST
    pbest_avg = PBEST
    px = 0
    py = 0
    reach = 0
    fugitive = 0

    def __init__(self, id, Vx, Vy, x, y):
        self.id = id
        self.Vx = Vx
        self.Vy = Vy
        self.x = x
        self.y = y

        # Store track information
        self.list_x = []
        self.list_y = []

        # x and y in last iteration
        self.lastx = x
        self.lasty = y



'''
Obstacle class
    id      : Obstacle id
    x       : Obstacle x value
    y       : Obstacle y value
    length  : Obstacle length
    width   : Obstacle width
'''
class Obstacle:
    def __init__(self, id, x, y, length, width):
        self.id = id
        self.x = x
        self.y = y
        self.length = length
        self.width = width



'''
Goal class
    id      : Goal id
    px      : Goal x value
    py      : Goal y value
'''
class Goal:
    px = 0
    py = 0

    def __init__(self, id, px, py):
        self.id = id
        self.px = px
        self.py = py



'''
function : detect obstacle
    detect the obstacle within Safe distance

    input:
    x : x position
    y : y position
    Obstacle : Obstacle instance

    return:
    out = 1 : detect obstacle
    out = 0 : no obstacle
'''
def detect_ob(x, y, Obstacle):
    out = 0

    if Obstacle.x - save < x < Obstacle.x + Obstacle.length + save and Obstacle.y - Obstacle.width - save < y < Obstacle.y + save:
        out = 1
    if out == 1:
        if can_print == 1:
            print("{} {}".format(x, y))
            print("{} {} {} {}".format(Obstacle.x, Obstacle.y, Obstacle.width, Obstacle.length))
    return out



'''
function : detect robot
    detect the robot within Safe distance

    input:
    x : x position
    y : y position
    ob : robot instance

    return:
    out = 1 : detect robot
    out = 0 : no obstacle
'''
def detect_ob_self(x, y, ob):
    out = 0

    if (x == ob.x and y == ob.y):
        return 0

    if abs(x - ob.x) < save and abs(y - ob.y) < save:
        out = 1
    return out



'''
function getFitness
    calculate the fitness value in this x,y position

    input: 
    gx      : x position of one goal
    gy      : y position of one goal
    x       : x positoin 
    y       : y position

    return:
    the Square root of distance between x,y and gx,gy
'''
def getFitness(gx, gy, x, y):
    dis = math.sqrt((gx - x) * (gx - x) + (gy - y) * (gy - y))
    sq = math.sqrt(dis)

    return math.sqrt(sq)