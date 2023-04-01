import random
import time

import numpy as np

from Astar import *
from Params import *

random.seed(RAND_SEED)


# print_to_txt(string: str): 这个函数定义了一个空函数，似乎是为了将一些信息打印到指定的文本文件中。
# get_distance(x1, x2, y1, y2): 这个函数用于计算两点之间的欧几里得距离，输入参数是两个点的坐标。
# get_theta(x1, x2, y1, y2): 这个函数用于计算两点之间的极角（弧度制），输入参数是两个点的坐标。
# get_angle(_theta, theta): 这个函数用于计算角度差，即第二个角度减去第一个角度后的差值，其中输入参数是两个弧度制的角度。如果角度差值在一定范围内，则将其视为0，否则按照正负值返回差值的绝对值。
def print_to_txt(string: str, i):
    if i == 0:
        with open("./log.txt", "w") as f:
            f.write(str(string) + '\n')
    else:
        with open("./log.txt", "a") as f:
            f.write(str(string) + '\n')


def get_distance(x1, x2, y1, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def get_theta(x1, x2, y1, y2):
    if x2 - x1 > 0:
        tan = (y2 - y1) / (x2 - x1)
        theta = np.arctan(tan)
    elif x2 - x1 == 0:
        theta = np.sign(y2 - y1) * np.pi / 2
    else:
        tan = (y2 - y1) / (x2 - x1)
        if y2 - y1 > 0:
            theta = np.pi + np.arctan(tan)
        else:
            theta = -np.pi + np.arctan(tan)
    return theta


def get_angle(_theta, theta):
    angle_diff = theta - _theta
    if abs(angle_diff) < np.pi / 180 or abs(angle_diff -
                                            2 * np.pi) < np.pi / 180:
        angle = 0
    elif abs(angle_diff) > np.pi:
        if angle_diff > 0:
            angle = 2 * np.pi - abs(angle_diff)
        else:
            angle = abs(angle_diff) - 2 * np.pi
    elif abs(angle_diff) <= np.pi:
        if angle_diff > 0:
            angle = -abs(angle_diff)
        else:
            angle = abs(angle_diff)
    return -angle


#判断是否同向
def is_syntropy(direction1, direction2, rx, ry, rspeed_x, rspeed_y, r_x, r_y,
                r_speed_x, r_speed_y):
    if direction1 < direction2:
        direction1, direction2 = direction2, direction1
    if direction1 - direction2 < np.pi * 0.15 or (direction1 > np.pi * 0.85 and
                                                  direction2 < -np.pi * 0.85):
        # 机器人1的位置和速度向量
        pos1 = np.array([rx, ry])
        vel1 = np.array([rspeed_x, rspeed_y])

        # 机器人2的位置和速度向量
        pos2 = np.array([r_x, r_x])
        vel2 = np.array([r_speed_x, r_speed_y])

        # 计算相对速度向量
        rel_vel = vel1 - vel2

        # 计算位置向量
        pos_diff = pos1 - pos2

        # 计算点积
        dot_product = np.dot(rel_vel, pos_diff)
        if dot_product > 0:
            return True
    return False


#判断是否反向
def is_opposite(direction1, direction2, rx, ry, rspeed_x, rspeed_y, r_x, r_y,
                r_speed_x, r_speed_y):
    if direction1 < direction2:
        direction1, direction2 = direction2, direction1
    if abs(direction1 - direction2 - np.pi) < np.pi / 6:
        # 机器人1的位置和速度向量
        pos1 = np.array([rx, ry])
        vel1 = np.array([rspeed_x, rspeed_y])

        # 机器人2的位置和速度向量
        pos2 = np.array([r_x, r_x])
        vel2 = np.array([r_speed_x, r_speed_y])

        # 计算相对速度向量
        rel_vel = -vel1 + vel2

        # 计算位置向量
        pos_diff = pos1 - pos2

        # 计算点积
        dot_product = np.dot(rel_vel, pos_diff)
        if dot_product > 0:
            return True
    return False


def is_boundary(x, y, direction):
    if (x < 2.5 and (direction > np.pi * 0.7 or direction < -np.pi * 0.7)):
        return True
    if (x > 47.5 and direction < np.pi * 0.3 and direction > -np.pi * 0.3):
        return True
    if (y < 2.5 and (direction > -np.pi * 0.8 and direction < -np.pi * 0.2)):
        return True
    if (y > 47.5 and direction < np.pi * 0.8 and direction > np.pi * 0.2):
        return True
    return False


def is_collide(distance, ty1, ty2):
    r1 = ROBO_RADIUS_NORM
    if ty1:
        r1 = ROBO_RADIUS_FULL
    r2 = ROBO_RADIUS_NORM
    if ty2:
        r2 = ROBO_RADIUS_FULL
    if distance < r1 + r2 + 0.2:
        return True
    return False


# 这是一个名为Handle的类，它代表了一个可操作的物体，包含一些属性和方法：

# 属性：

# id: 物体的唯一标识符
# handle_type: 物体的类型
# x, y: 物体的坐标
# left_time: 物体离开的剩余时间
# material: 物体上的物料
# object: 物体上的对象
# is_assigned_pickup: 物体是否已被指派
# material_onroute: 物体正在路上的物料
# 方法：

# update: 更新物体的属性
# identify_short_material: 识别缺少的物料
# 在update方法中，给定物体的类型、坐标、离开时间、物料和对象，更新物体的属性。


# 在identify_short_material方法中，将物体上缺少的物料识别出来。
class Handle:

    def __init__(self, id) -> None:
        self.id = id
        self.handle_type = 0
        self.x = 0
        self.y = 0
        self.left_time = -1
        self.material = 0
        self.object = 0

        self.is_assigned_pickup = 0
        self.material_onroute = np.zeros(HANDLE_TYPE_NUM)

    def update(self, handle_type, x, y, left_time, material, object):
        self.handle_type = handle_type
        self.x = x
        self.y = y
        self.left_time = left_time
        self.material = material
        self.object = object
        self.identify_short_material()

    def identify_short_material(self):
        material_info = "{:08b}".format(self.material)
        self.material_shortage = []
        for m in TYPE_MATERIAL[self.handle_type]:
            if int(material_info[HANDLE_OBJECT_NUM -
                                 m]) + self.material_onroute[m - 1] == 0:
                self.material_shortage.append(m)


# 该类包含以下属性：

# id：机器人的标识符。
# delta_time：更新之间的时间差，根据一个名为FPS的常量计算得出。
# handle：对机器人的引用。
# object_type：机器人物品类型的标识符。
# time_coeff：时间系数。
# crash_coeff：碰撞系数。
# rotate_speed：机器人旋转速度。
# speed_x：机器人沿x轴移动的速度。
# speed_y：机器人沿y轴移动的速度。
# direction：机器人面朝的方向。
# x：机器人当前位置的x坐标。
# y：机器人当前位置的y坐标。
# is_assigned_task：标志机器人是否有任务分配。
# x_：任务的x坐标。
# y_：任务的y坐标。
# task_list：机器人要完成的任务列表。
# strategy_dict：机器人的策略字典。
# last_assigned_time：机器人最后一次接收到任务的时间。
# 该类还包含以下方法：

# __init__：初始化机器人的属性。
# update：更新机器人的属性。
# add_task：向机器人的任务列表中添加一个新的任务。
# strategy：确定机器人的策略。
# arrive：机器人到达任务位置时执行的操作。


class Robot:

    def __init__(self, id) -> None:
        self.id = id
        self.delta_time = 1 / FPS
        self.handle = None
        self.object_type = 0
        self.time_coeff = 0
        self.crash_coeff = 0
        self.rotate_speed = 0
        self.speed_x = 0
        self.speed_y = 0
        self.direction = 0
        self.x = 0
        self.y = 0
        self.is_assigned_task = 0
        self.x_ = None
        self.y_ = None
        self.task_list = []
        self.strategy_dict = {}
        self.last_assigned_time = 0
        self.last_assigned_time
        #用来判断机器人是否处在避让状态
        self.is_avoid = False
        #判断机器人是否有规划好的路径
        self.is_plan = False
        #存储机器人路径
        self.path = []
        self.path_x = None
        self.path_y = None

    def get_speed(self):
        return np.sqrt(self.speed_x**2 + self.speed_y**2)

    def set_speed(self, speed):
        if not self.is_avoid:
            if speed > 6:
                self.strategy_dict[0] = 6
            elif speed < -2:
                self.strategy_dict[0] = -2
            else:
                self.strategy_dict[0] = speed

    def set_rotate(self, rotate):
        if not self.is_avoid:
            if rotate > np.pi:
                self.strategy_dict[0] = np.pi
            elif rotate < -np.pi:
                self.strategy_dict[0] = -np.pi
            else:
                self.strategy_dict[1] = rotate

    def attractive_force(self):
        if self.is_assigned_task:
            dx = self.x_ - self.x
            dy = self.y_ - self.y
            d = np.sqrt(dx**2 + dy**2)
            if d < ROBO_HANDLE_DIST * 8:
                fx = dx / d * (8 * ROBO_HANDLE_DIST / d -
                               ROBO_HANDLE_DIST) + 0.8
                fy = dy / d * (8 * ROBO_HANDLE_DIST / d -
                               ROBO_HANDLE_DIST) + 0.8
                return fx, fy
            else:
                return 0, 0
        return 0, 0

    def attractive_force2(self):
        # if self.is_assigned_task:
        #     dx = self.x_ - self.x
        #     dy = self.y_ - self.y
        #     d = np.sqrt(dx**2 + dy**2)
        #     if d < ROBO_HANDLE_DIST * 8:
        #         fx = dx / d * (8 * ROBO_HANDLE_DIST / d -
        #                        ROBO_HANDLE_DIST) + 0.8
        #         fy = dy / d * (8 * ROBO_HANDLE_DIST / d -
        #                        ROBO_HANDLE_DIST) + 0.8
        #         return fx, fy
        #     else:
        #         return 0, 0
        return 0, 0

    def update(self, handle, object_type, time_coeff, crash_coeff,
               rotate_speed, speed_x, speed_y, direction, x, y):
        self.handle: Handle = handle
        self.object_type = object_type
        self.time_coeff = time_coeff
        self.crash_coeff = crash_coeff
        self.rotate_speed = rotate_speed
        self.speed_x = speed_x
        self.speed_y = speed_y
        self.direction = direction
        self.x = x
        self.y = y

    def add_task(self, x, y, todo_type, frame):
        if todo_type in [BUY, SELL]:
            self.is_assigned_task = 1
            self.last_assigned_time = frame
            if len(self.task_list) == 0:
                self.x_ = x
                self.y_ = y
                self.todo_type = todo_type
            self.task_list.append((x, y, todo_type))
        elif todo_type == DESTROY:
            self.is_assigned_task = 0
            self.x_ = x
            self.y_ = y
            self.todo_type = todo_type
            self.task_list.insert(0, (x, y, DESTROY))
        elif todo_type == GOTO:
            self.is_assigned_task = 0
            self.last_assigned_time = frame
            if len(self.task_list) == 0:
                self.x_ = x
                self.y_ = y
                self.todo_type = todo_type
            self.task_list.append((x, y, todo_type))

    def strategy(self):
        self.strategy_dict = {}

        if self.x_ is None or self.y_ is None:
            stop_strategy = {FORWARD: 0, ROTATE: 0}
            self.strategy_dict = stop_strategy
            return

        distance = get_distance(self.x, self.x_, self.y, self.y_)
        if distance < ROBO_HANDLE_DIST:
            stop_strategy = {FORWARD: 0, ROTATE: 0, self.arrive(): -1}
            self.strategy_dict = stop_strategy
            return

        theta = get_theta(self.x, self.x_, self.y, self.y_)
        angle = get_angle(self.direction, theta)

        speed = min(distance / self.delta_time, MAX_FORE_SPEED)
        if abs(angle) > np.pi / 2 - 0.1:
            if distance < 8 * ROBO_HANDLE_DIST:
                speed = 0
                self.is_avoid = True
            else:
                speed = 2
        if distance < 4 * ROBO_HANDLE_DIST:
            if abs(angle) > np.pi / 4:
                speed = distance / 2 * ROBO_HANDLE_DIST
                self.is_avoid = True
        rotate_speed = min(abs(angle) / self.delta_time, MAX_ROTATE_SPEED)
        self.strategy_dict = {
            FORWARD: speed,
            ROTATE: np.sign(angle) * rotate_speed
        }

    def strategy2(self):
        self.strategy_dict = {}

        if self.x_ is None or self.y_ is None:
            stop_strategy = {FORWARD: 0, ROTATE: 0}
            self.strategy_dict = stop_strategy
            return

        distance = get_distance(self.x, self.x_, self.y, self.y_)
        if distance < ROBO_HANDLE_DIST:
            stop_strategy = {FORWARD: 0, ROTATE: 0, self.arrive(): -1}
            self.strategy_dict = stop_strategy
            return

        theta = get_theta(self.x, self.x_, self.y, self.y_)
        angle = get_angle(self.direction, theta)

        speed = min(distance / self.delta_time, MAX_FORE_SPEED)
        if abs(angle) > np.pi / 2 - 0.1:
            if distance < 8 * ROBO_HANDLE_DIST:
                speed = 2 * distance / 8 / ROBO_HANDLE_DIST
                self.is_avoid = True
            else:
                speed = 2 * distance / 8 / ROBO_HANDLE_DIST + 2
        if distance < 4 * ROBO_HANDLE_DIST:
            if abs(angle) > np.pi / 4:
                speed = distance / 2 * ROBO_HANDLE_DIST
                self.is_avoid = True
        rotate_speed = min(abs(angle) / self.delta_time, MAX_ROTATE_SPEED)
        self.strategy_dict = {
            FORWARD: speed,
            ROTATE: np.sign(angle) * rotate_speed
        }

    #轨迹跟踪规划
    def strategy_path(self):
        self.strategy_dict = {}
        if len(self.path) > 0:
            self.path_x, self.path_y = self.path[0]  #可优化
        if self.x_ is None or self.y_ is None or len(self.path) == 0:
            stop_strategy = {FORWARD: 0, ROTATE: 0}
            self.strategy_dict = stop_strategy
            return
        distance = get_distance(self.x, self.path_x, self.y, self.path_y)
        distance2 = get_distance(self.x, self.x_, self.y, self.y_)
        # 如果已经到达目的地则将小车变为未规划状态
        if distance2 < ROBO_HANDLE_DIST:
            stop_strategy = {FORWARD: 0, ROTATE: 0, self.arrive(): -1}
            self.strategy_dict = stop_strategy
            self.is_plan = False
            return
        #如果到达路径点则更新到下一个路径点并在path中删除已经到达的路径点
        if distance < ROBO_HANDLE_DIST:
            self.path.pop(0)
            if len(self.path) > 0:
                self.path_x, self.path_y = self.path[0]  #可优化

        theta = get_theta(self.x, self.path_x, self.y, self.path_y)
        angle = get_angle(self.direction, theta)

        speed = min(distance / self.delta_time, MAX_FORE_SPEED)
        if abs(angle) > np.pi / 2 - 0.1:
            speed = 0
        if abs(angle) > np.pi / 4:
            speed = 0
        rotate_speed = min(abs(angle) / self.delta_time, MAX_ROTATE_SPEED)
        self.strategy_dict = {
            FORWARD: speed,
            ROTATE: np.sign(angle) * rotate_speed
        }

    def arrive(self):
        if self.todo_type == BUY:
            self.handle.is_assigned_pickup = 0
        elif self.todo_type == SELL:
            self.handle.material_onroute[self.object_type - 1] -= 1

        pretask = self.task_list.pop(0)
        if len(self.task_list) == 0:
            self.is_assigned_task = 0
            self.x_ = None
            self.y_ = None
            self.todo_type = 0
        else:
            task = self.task_list[0]
            self.x_ = task[0]
            self.y_ = task[1]
            self.todo_type = task[2]
        return pretask[2]


# __init__(self): 类的初始化函数，设置了一些基本的地图属性，如当前帧数、初始金钱、机器人列表、抓取器列表、抓取器类型字典等。
# update_map(self, frame, money): 更新地图的帧数和金钱。
# init_robots(self): 初始化机器人列表。
# init_handles(self, num): 初始化抓取器列表。
# update_robot(self, id, handle_id, object_type, time_coeff, crash_coeff, rotate_speed, speed_x, speed_y, direction, x, y): 更新机器人的状态。
# update_handle(self, id, handle_type, x, y, left_time, material, object): 更新抓取器的状态。
# update_handle_type_dict_first(self): 根据抓取器类型将抓取器列表分类。
# strategy_to_str(self, robot: Robot): 将机器人的策略转化为字符串格式。
# output_strategy(self): 输出机器人的策略。
# get_short_material(self): 获取缺货的原材料列表。
# set_robots_targets(self): 设置机器人的任务和目标位置。
# 其中，类变量包括：


# frame: 当前帧数。
# money: 当前金钱。
# robot_list: 机器人列表。
# handle_list: 抓取器列表。
# handle_type_dict: 抓取器类型字典。
class Map:

    def __init__(self) -> None:
        self.frame = 0
        self.money = INIT_MONEY
        self.robot_list = []
        self.handle_list = []
        self.handle_type_dict = {}
        self.choose = 2
        self.ox = []
        self.oy = []
        self.a_star = AStarPlanner(MAP1_OX, MAP1_OY, 1, 0.5)
        self.map_type = 1

    def update_map(self, frame, money):
        self.frame = frame
        self.money = money

    def init_robots(self):
        for i in range(ROBO_NUM):
            self.robot_list.append(Robot(id=i))

    def init_handles(self, num):
        if num == 37:
            self.map_type = 1
        elif num == 8:
            self.map_type = 2
            self.a_star = AStarPlanner(MAP2_OX, MAP2_OY, 0.6, 1)
        elif num == 13:
            self.map_type = 3
            self.a_star = AStarPlanner(MAP3_OX, MAP3_OY, 1, 0.5)
        else:
            self.map_type = 4
            self.a_star = AStarPlanner(MAP4_OX, MAP4_OY, 1, 0.5)
        for i in range(num):
            self.handle_list.append(Handle(id=i))

    def update_robot(self, id, handle_id, object_type, time_coeff, crash_coeff,
                     rotate_speed, speed_x, speed_y, direction, x, y):
        try:
            h = None
            if handle_id != -1:
                h = self.handle_list[handle_id]
            self.robot_list[id].update(h, object_type, time_coeff, crash_coeff,
                                       rotate_speed, speed_x, speed_y,
                                       direction, x, y)
        except:
            pass
            # print_to_txt(str((len(self.robot_list), id)))

    def update_handle(self, id, handle_type, x, y, left_time, material,
                      object):
        try:
            self.handle_list[id].update(handle_type, x, y, left_time, material,
                                        object)
        except:
            pass
            # print_to_txt(str((len(self.handle_list), id)))

    def update_handle_type_dict_first(self):
        for i in range(HANDLE_TYPE_NUM):
            self.handle_type_dict[i + 1] = []
        for h in self.handle_list:
            h: Handle
            self.handle_type_dict[h.handle_type].append(h)

    def strategy_to_str(self, robot: Robot):
        strategy_str = ""
        for s, p in robot.strategy_dict.items():
            if s == FORWARD:
                strategy_str += "forward {:.0f} {:}\n".format(robot.id, p)
            elif s == ROTATE:
                strategy_str += "rotate {:.0f} {:}\n".format(robot.id, p)
            elif s == BUY:
                strategy_str += "buy {:.0f}\n".format(robot.id)
            elif s == SELL:
                strategy_str += "sell {:.0f}\n".format(robot.id)
            elif s == DESTROY:
                strategy_str += "destroy {:.0f}\n".format(robot.id)
        return strategy_str

    def output_strategy(self):
        outputs = ""
        # print_to_txt('frame: ' + str(self.frame), 1)
        for r in self.robot_list:
            # print_to_txt(r.id, 1)
            #如果有任务且还没有进行路径规划则进行路径规划
            if r.is_assigned_task != 0 and r.is_plan == False:
                # print_to_txt(
                #     str(r.x) + ' ' + str(r.y) + ' ' + str(r.x_) + ' ' +
                #     str(r.y_), 1)
                r.path = self.a_star.planning(r.x, r.y, r.x_, r.y_)
                r.is_plan = True
                # print_to_txt(r.path, 1)
            r.is_avoid = False
            if r.y > 48:
                r.is_avoid = True
            #这里应当修改为沿路径行驶，还未修改。
            r.strategy_path()
        # print_to_txt('final', 1)
        for r in self.robot_list:
            #如果靠近边缘则限制最大速度
            if r.get_speed() > 4:
                r.set_speed(4)
            # if r.is_recede:
            #     r.is_recede -= 1
            output = self.strategy_to_str(r)
            if output == "":
                continue
            outputs += output
        return outputs

    def output_strategy2(self):
        outputs = ""
        for r in self.robot_list:
            r.is_avoid = False
            r.strategy2()
        #用人工势场方法进行防撞修正
        for i in range(ROBO_NUM):
            r: Robot = self.robot_list[i]
            #计算合力
            robot_force_x = 0
            robot_force_y = 0
            #计算到目标点的引力
            aforce_x, aforce_y = aforce_x, aforce_y = r.attractive_force2()
            #计算到其他机器人的斥力
            for j in range(ROBO_NUM):
                if i != j:
                    r_: Robot = self.robot_list[j]
                    distance = get_distance(r.x, r_.x, r.y, r_.y)
                    if distance < 3 * ROBO_RADIUS_FULL * r.get_speed() / 3 + 3:
                        #计算到其他机器人的斥力
                        dx = r.x - r_.x
                        dy = r.y - r_.y
                        robot_force_x += 10 * ROBO_RADIUS_FULL * dx / distance**2
                        robot_force_y += 10 * ROBO_RADIUS_FULL * dy / distance**2
            #计算合力向量，并根据现在机器人速度和合力夹角给出调整
            # print_to_txt(
            #     str(self.frame) + " id:" + str(r.id) + " force:" +
            #     str(robot_force_x) + " " + str(robot_force_y))
            force = np.array(
                [robot_force_x + aforce_x, robot_force_y + aforce_y])
            dir1 = np.array([np.cos(r.direction), np.sin(r.direction)])
            # print_to_txt("dir " + str(dir1) + " " + str(r.direction))
            force_val = np.linalg.norm(force)
            if force_val > 2:
                cos_angle = np.dot(force, dir1) / force_val
                angle = np.arccos(cos_angle)
                cross = -np.cross(force, dir1)
                # print_to_txt("angle:" + str(angle) + " " + str(cross))
                force
                if angle > np.pi * 0.7:
                    if force_val > 4:
                        if r.object_type:
                            r.set_speed(r.get_speed() - force_val / 4)
                        else:
                            r.set_speed(r.get_speed() - force_val / 2)
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + force_val * np.pi / 10)
                    # print_to_txt("set " + str(r.id) + " 1")
                    if cross <= 0:
                        r.set_rotate(r.rotate_speed - force_val * np.pi / 10)
                elif angle < np.pi * 0.3:
                    if force_val:
                        r.set_speed(r.get_speed() + force_val / 4)
                else:
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + np.pi / 10)
                    # print_to_txt("set " + str(r.id) + " 1")
                    if cross <= 0:
                        r.set_rotate(r.rotate_speed - np.pi / 10)

        for r in self.robot_list:
            #如果靠近边缘则限制最大速度
            if is_boundary(r.x, r.y, r.direction) and r.get_speed() > 3:
                r.set_speed(3)
            # if r.is_recede:
            #     r.is_recede -= 1
            output = self.strategy_to_str(r)
            if output == "":
                continue
            outputs += output
        return outputs

    def get_short_material(self):
        short_material = set()
        for h in self.handle_list:
            h: Handle
            short_material = short_material.union(h.material_shortage)
        return short_material

    def set_robots_targets(self):
        pickup_tasks = [[] for i in range(HANDLE_OBJECT_NUM)]
        delivery_tasks = {}
        delivery_edges = {}
        for h in self.handle_list:
            h: Handle
            if h.object == 1 and h.is_assigned_pickup == 0:
                pickup_tasks[h.handle_type - 1].append(h)
        for h in self.handle_list:
            short_material = h.material_shortage
            if len(short_material) > 0:
                delivery_tasks[h.id] = []
                delivery_tasks_h = delivery_tasks[h.id]
                avg_revenue = (
                    SELL_PRICE[h.handle_type - 1] -
                    BUY_PRICE[h.handle_type - 1]) / len(short_material)
                for m in short_material:
                    delivery_tasks_h.append(m)
                    for h_ in pickup_tasks[m - 1]:
                        delivery_edges.setdefault(h_, list()).append(
                            (h, avg_revenue,
                             get_distance(h_.x, h.x, h_.y, h.y)))

        delivery_origins = list(delivery_edges.keys())
        for r in self.robot_list:
            r: Robot
            if r.is_assigned_task == 0:
                if r.object_type == 0:
                    if len(delivery_origins) <= 0:
                        continue
                    left_frame = TOTAL_TIME * 60 * FPS - self.frame
                    left_max_distance = left_frame / FPS * MAX_FORE_SPEED
                    # if left_max_distance < MAP_SIZE * 2:
                    #     r.add_task(MAP_SIZE, MAP_SIZE, GOTO, self.frame)
                    #     continue
                    distance_list = [
                        get_distance(r.x, h_.x, r.y, h_.y) / MAX_FORE_SPEED *
                        STORE_COST[h_.handle_type - 1]
                        for h_ in delivery_origins
                    ]
                    h_idx = np.argmin(distance_list)
                    h = delivery_origins[h_idx]

                    revenue_list = [
                        -(i[1] - i[2] / MAX_FORE_SPEED *
                          STORE_COST[i[0].handle_type - 1])
                        for i in delivery_edges[h]
                    ]
                    for h__idx in np.argsort(revenue_list):
                        h_: Handle = delivery_edges[h][h__idx][0]
                        if h.handle_type in h_.material_shortage:
                            break
                    if get_distance(r.x, h.x, r.y, h.y) + get_distance(
                            h.x, h_.x, h.y, h_.y) < left_max_distance * 0.8:
                        r.add_task(h.x, h.y, BUY, self.frame)
                        h.is_assigned_pickup = 1
                        delivery_origins.pop(h_idx)
                        r.add_task(h_.x, h_.y, SELL, self.frame)
                        #防止同时操作导致找不到要清除的元素
                        if h.handle_type in h_.material_shortage:
                            h_.material_shortage.remove(h.handle_type)
                        h_.material_onroute[h.handle_type - 1] += 1
                        delivery_edges[h].pop(h__idx)
                    else:
                        r.add_task(MAP_SIZE, MAP_SIZE, GOTO, self.frame)
                    # type_list = list(self.get_short_material())
                    # for t in random.sample(type_list, len(type_list)):
                    #     for h in random.sample(self.handle_type_dict[t], len(self.handle_type_dict[t])):
                    #         h: Handle
                    #         if h.object == 1 and h.is_assigned_pickup == 0:
                    #             r.add_task(h.x, h.y, 2, self.frame)
                    #             h.is_assigned_pickup = 1
                    #             break
                    #     if r.is_assigned_task == 1:
                    #         break
                else:
                    for t in random.sample(MATERIAL_TYPE[r.object_type],
                                           len(MATERIAL_TYPE[r.object_type])):
                        for h in random.sample(self.handle_type_dict[t],
                                               len(self.handle_type_dict[t])):
                            h: Handle
                            if r.object_type in h.material_shortage:
                                r.add_task(h.x, h.y, 3, self.frame)
                                h.material_shortage.remove(r.object_type)
                                h.material_onroute[r.object_type - 1] += 1
                                break
                        if r.is_assigned_task == 1:
                            break
                    if r.is_assigned_task == 0:
                        r.add_task(r.x, r.y, DESTROY, self.frame)
                        # print_to_txt("no where to go")
