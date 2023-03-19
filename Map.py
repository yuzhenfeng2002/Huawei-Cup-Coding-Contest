from Params import *
import numpy as np
import random

random.seed(RAND_SEED)


# print_to_txt(string: str): 这个函数定义了一个空函数，似乎是为了将一些信息打印到指定的文本文件中。
# get_distance(x1, x2, y1, y2): 这个函数用于计算两点之间的欧几里得距离，输入参数是两个点的坐标。
# get_theta(x1, x2, y1, y2): 这个函数用于计算两点之间的极角（弧度制），输入参数是两个点的坐标。
# get_angle(_theta, theta): 这个函数用于计算角度差，即第二个角度减去第一个角度后的差值，其中输入参数是两个弧度制的角度。如果角度差值在一定范围内，则将其视为0，否则按照正负值返回差值的绝对值。
def print_to_txt(string: str):
    with open("./log.txt", "a") as f:
        f.write(str(string) + '\n')


def get_distance(x1, x2, y1, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_object_distance(h, h_):
    return get_distance(h.x, h_.x, h.y, h_.y)

def distance_to_time(distance, speed=MAX_FORE_SPEED*0.98):
    return distance / speed

def distance_to_frame(distance, speed=MAX_FORE_SPEED*0.98):
    return distance_to_time(distance, speed) * FPS

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
def is_syntropy(drection1, drection2):
    if drection1 < drection2:
        drection1, drection2 = drection2, drection1
    if drection1 - drection2 < np.pi * 0.15:
        return True
    elif drection1 > np.pi * 0.8 and drection2 < -np.pi * 0.8:
        return True
    return False


#判断是否反向
def is_opposite(direction1, direction2):
    if direction1 < direction2:
        direction1, direction2 = direction2, direction1
    if abs(direction1 - direction2 - np.pi) < np.pi / 7:
        return True
    return False


def is_boundary(x, y, direction):
    if (x < 2 and (direction > np.pi * 0.8 or direction < -np.pi * 0.8)):
        return True
    if (x > 48 and direction < np.pi * 0.2 and direction > -np.pi * 0.2):
        return True
    if (y < 2 and (direction > -np.pi * 0.7 and direction < -np.pi * 0.3)):
        return True
    if (y > 48 and direction < np.pi * 0.7 and direction < np.pi * 0.3):
        return True
    return False

#计算价值系数
def cal_value_coeff(x, maxX, minR):
    if x >= maxX: return minR
    else:
        return minR + (1-minR) * (
            1-np.sqrt(
            1-(1-x/maxX)**2
            )
        )

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
            if int(material_info[HANDLE_OBJECT_NUM - m]) + self.material_onroute[m - 1] == 0:
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
        #用来判断机器人是否处在避让状态
        self.is_avoid = False

        self.left_time_for_tasks = 0

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
        self.left_time_for_tasks = 0
        for x_,y_,_ in self.task_list:
            self.left_time_for_tasks += get_distance(x, x_, y, y_)
            x = x_
            y = y_

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
        if abs(angle) > np.pi / 2: speed = 0
        if abs(angle) > np.pi / 4 and distance < 4 * ROBO_HANDLE_DIST:
            speed = 1
        rotate_speed = min(abs(angle) / self.delta_time, MAX_ROTATE_SPEED)
        self.strategy_dict = {
            FORWARD: speed,
            ROTATE: np.sign(angle) * rotate_speed
        }

    def arrive(self):
        if self.todo_type == BUY:
            if self.handle.object != 1:
                return WAIT
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

    def update_map(self, frame, money):
        self.frame = frame
        self.money = money

    def init_robots(self):
        for i in range(ROBO_NUM):
            self.robot_list.append(Robot(id=i))

    def init_handles(self, num):
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
        for r in self.robot_list:
            r.strategy()
        for i in range(ROBO_NUM - 1):
            r: Robot = self.robot_list[i]
            for j in range(i + 1, ROBO_NUM):
                r_: Robot = self.robot_list[j]
                #靠近
                if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 6
                        and (is_opposite(r.direction, r_.direction)
                             or is_syntropy(r.direction, r_.direction))):
                    theta = get_theta(r.x, r_.x, r.y, r_.y)
                    angle = get_angle(r.direction, theta)
                    rotate_speed = -np.sign(angle) * np.pi
                    if get_distance(r.x, r_.x, r.y,
                                    r_.y) < ROBO_RADIUS_FULL * 3:
                        if r_.is_avoid == False:
                            r.strategy_dict[1] = rotate_speed
                            r.strategy_dict[0] = 2
                            r.is_avoid = True
                    else:
                        r.strategy_dict[1] = rotate_speed * 0.6
                        r.is_avoid = False
                #
        for r in self.robot_list:
            #如果靠近边缘则限制最大速度
            if is_boundary(r.x, r.y, r.direction) and r.strategy_dict[0] > 4:
                r.strategy_dict[0] = 4
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
    
    def cal_pickup_and_delivery_utility(self, pickup_time, delivery_time, reward, r, h, h_):
        # if len(self.handle_list) >= 25:
        #     h_type = h_.handle_type - 1
        #     reward2 = (SELL_PRICE[h_type]-BUY_PRICE[h_type]) / len(h_.material_shortage)
        #     return (reward) / (pickup_time+delivery_time)
        # else:
            return (reward) / (pickup_time+delivery_time
                            #    +distance_to_time(get_distance(h_.x,MAP_SIZE/2,h_.y,MAP_SIZE/2))
                               )

    def set_robots_targets(self):
        feasible_pickup_edges = {r:{} for r in self.robot_list}
        feasible_delivery_edges = {h:{} for h in self.handle_list}
        for h in self.handle_list:
            h: Handle
            if h.handle_type > HANDLE_OBJECT_NUM or h.left_time < 0 or h.is_assigned_pickup == 1:
                continue
            for h__type_id in MATERIAL_TYPE[h.handle_type]:
                h__list = self.handle_type_dict[h__type_id]
                for h_ in h__list:
                    h_: Handle
                    if h.handle_type in h_.material_shortage:
                        feasible_delivery_edges[h][h_] = (
                            SELL_PRICE[h.handle_type-1], distance_to_time(get_object_distance(h, h_)),
                            SELL_PRICE[h.handle_type-1]*cal_value_coeff(distance_to_frame(get_object_distance(h, h_)), 9000, 0.8) - BUY_PRICE[h.handle_type-1] 
                            # + (SELL_PRICE[h_.handle_type-1]-BUY_PRICE[h_.handle_type-1]) / len(h_.material_shortage)
                        )
            if len(feasible_delivery_edges[h]) != 0:
                for r in self.robot_list:
                    if h.left_time/FPS - distance_to_time(get_object_distance(h, r)) >= 5:
                        continue
                    feasible_pickup_edges[r][h] = (
                        -BUY_PRICE[h.handle_type-1], 
                        max(distance_to_time(get_object_distance(h, r)), h.left_time/FPS)
                    )
        

        left_frame = TOTAL_TIME * 60 * FPS - self.frame
        left_max_distance = left_frame / FPS * MAX_FORE_SPEED
        for r in self.robot_list:
            r: Robot
            if r.is_assigned_task == 0:
                if r.object_type == 0:
                    if len(feasible_pickup_edges[r]) <= 0:
                        continue
                    utility = -BIG_M
                    hbest = None
                    h_best = None
                    for h, pickup_edge_info in feasible_pickup_edges[r].items():
                        pickup_time = pickup_edge_info[1]
                        for h_, delivery_edge_info in feasible_delivery_edges[h].items():
                            if h.handle_type not in h_.material_shortage:
                                continue
                            delivery_time = delivery_edge_info[1]
                            reward = delivery_edge_info[2]
                            if pickup_time + delivery_time >= left_frame / FPS * 0.8:
                                continue
                            u = self.cal_pickup_and_delivery_utility(pickup_time, delivery_time, reward, r, h, h_)
                            if u > utility:
                                hbest = h
                                h_best = h_
                                utility = u
                    h = hbest
                    h_ = h_best
                    if not(h is None or h_ is None):
                        r.add_task(h.x, h.y, BUY, self.frame)
                        h.is_assigned_pickup = 1
                        r.add_task(h_.x, h_.y, SELL, self.frame)
                        #防止同时操作导致找不到要清除的元素
                        if h.handle_type in h_.material_shortage:
                            h_.material_shortage.remove(h.handle_type)
                        h_.material_onroute[h.handle_type - 1] += 1
                        feasible_delivery_edges[h].pop(h_)
                        if len(feasible_delivery_edges) == 0:
                            for r_ in self.robot_list:
                                feasible_pickup_edges[r_].pop(h)
                    else:
                        r.add_task(MAP_SIZE, MAP_SIZE, GOTO, self.frame)
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
                        r.add_task(r.x, r.y, 4, self.frame)
