import random

import numpy as np

from b_Astar import *
from Params import *
from Robot import *
from Handle import *
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
        self.a_star = BidirectionalAStarPlanner(MAP1_OX, MAP1_OY, 0.6, 0.6)
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
            self.a_star = BidirectionalAStarPlanner2(MAP2_OX, MAP2_OY, 0.5, 0.7)
        elif num == 13:
            self.map_type = 3
            self.a_star = BidirectionalAStarPlanner(MAP3_OX, MAP3_OY, 0.6, 1)
        else:
            self.map_type = 4
            self.a_star = BidirectionalAStarPlanner2(MAP4_OX, MAP4_OY, 1, 0.7)
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
                # if (len(r.path) == 1):
                #     r.arrive()
                #     break
                r.is_plan = True
                # print_to_txt(r.path, 1)
            r.is_avoid = False
            #沿路径行驶。
            r.strategy_path()
        # for i in range(ROBO_NUM-1):
        #     r: Robot = self.robot_list[i]
        #     for j in range(i+1, ROBO_NUM):
        #         r_: Robot = self.robot_list[j]
        #         if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 10 and 
        #             abs(abs(r.direction - r_.direction)-np.pi) < np.pi/6):
        #             theta = get_theta(r.x, r_.x, r.y, r_.y)
        #             angle = get_angle(r.direction, theta)
        #             rotate_speed = -np.sign(angle) * np.pi
        #             r.strategy_dict[1] = rotate_speed
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
        # print_to_txt(self.frame, 1)
        # print_to_txt(outputs, 1)
        return outputs
    def output_strategy2(self):
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
                # if (len(r.path) == 1):
                #     r.arrive()
                #     break
                r.is_plan = True
                # print_to_txt(r.path, 1)
            r.is_avoid = False
            #沿路径行驶。
            r.strategy_path()
        for i in range(ROBO_NUM-1):
            r: Robot = self.robot_list[i]
            for j in range(i+1, ROBO_NUM):
                r_: Robot = self.robot_list[j]
                if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 10 and 
                    abs(abs(r.direction - r_.direction)-np.pi) < np.pi/6):
                    theta = get_theta(r.x, r_.x, r.y, r_.y)
                    angle = get_angle(r.direction, theta)
                    rotate_speed = -np.sign(angle) * np.pi
                    r.strategy_dict[1] = rotate_speed
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
        # print_to_txt(self.frame, 1)
        # print_to_txt(outputs, 1)
        return outputs
    def output_strategy4(self):
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
                # if (len(r.path) == 1):
                #     r.arrive()
                #     break
                r.is_plan = True
                # print_to_txt(r.path, 1)
            r.is_avoid = False
            #沿路径行驶。
            r.strategy_path4()
        for i in range(ROBO_NUM-1):
            r: Robot = self.robot_list[i]
            for j in range(i+1, ROBO_NUM):
                r_: Robot = self.robot_list[j]
                if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 10 and 
                    abs(abs(r.direction - r_.direction)-np.pi) < np.pi/6):
                    theta = get_theta(r.x, r_.x, r.y, r_.y)
                    angle = get_angle(r.direction, theta)
                    rotate_speed = -np.sign(angle) * np.pi
                    r.strategy_dict[1] = rotate_speed
        for r in self.robot_list:
            #限制最大速度
            if r.get_speed() > 4:
                r.set_speed(4)
            # if r.is_recede:
            #     r.is_recede -= 1
            output = self.strategy_to_str(r)
            if output == "":
                continue
            outputs += output
        # print_to_txt(self.frame, 1)
        # print_to_txt(outputs, 1)
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
