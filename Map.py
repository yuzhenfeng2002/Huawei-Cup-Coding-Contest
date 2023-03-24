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
        # 是否发生碰撞
        self.is_colide = False
        # #判断是否接近目标位置
        # self.is_close = False

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
            self.strategy_dict[1] = rotate
#待完善

    def attractive_force(self):
        fx, fy = 0, 0
        boundary = 3
        f_boundary = 1.8
        if self.x > MAP_SIZE - boundary:
            fx -= f_boundary
        if self.x < boundary:
            fx += f_boundary
        if self.y > MAP_SIZE - boundary:
            fy -= f_boundary
        if self.y < boundary:
            fy += f_boundary
        # if self.is_assigned_task:
        #     dx = self.x_ - self.x
        #     dy = self.y_ - self.y
        #     d = np.sqrt(dx**2 + dy**2)
        #     # dir1 = np.array([np.cos(self.direction), np.sin(self.direction)])
        #     # d_ = np.array([dx, dy])
        #     # cos_angle = np.dot(d_, dir1) / np.linalg.norm(d_)
        #     # angle = np.arccos(cos_angle)
        #     # if angle > np.pi / 2:
        #     #     fx += 1.92 * angle * dx / abs(dx)
        #     #     fx += 1.92 * angle * dy / abs(dy)
        #     if d < 8 * ROBO_HANDLE_DIST and d > ROBO_HANDLE_DIST * 2:
        #         fx += 5 * dx / d - dx / ROBO_HANDLE_DIST
        #         fy += 5 * dy / d - dy / ROBO_HANDLE_DIST
        return fx, fy

    def set_speed2(self, speed):
        if not self.is_avoid:
            if speed > 6:
                self.strategy_dict[0] = 6
            elif speed < -2:
                self.strategy_dict[0] = -2
            else:
                self.strategy_dict[0] = speed

    def set_rotate2(self, rotate):
        if not self.is_avoid:
            if rotate > np.pi:
                self.strategy_dict[1] = np.pi
            elif rotate < -np.pi:
                self.strategy_dict[1] = -np.pi
            else:
                self.strategy_dict[1] = rotate

    def attractive_force2(self):
        if self.is_assigned_task:
            dx = self.x_ - self.x
            dy = self.y_ - self.y
            d = np.sqrt(dx**2 + dy**2)
            if d < ROBO_HANDLE_DIST * 8:
                fx = dx / d * (8 * ROBO_HANDLE_DIST / d - ROBO_HANDLE_DIST +
                               0.8)
                fy = dy / d * (8 * ROBO_HANDLE_DIST / d - ROBO_HANDLE_DIST +
                               0.8)
                return fx, fy
            else:
                return 0, 0
        return 0, 0

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
        if abs(angle) > np.pi / 2 - 0.1:
            if distance < 5 * ROBO_HANDLE_DIST:
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
        self.choose = 1

    def update_map(self, frame, money):
        self.frame = frame
        self.money = money

    def init_robots(self):
        for i in range(ROBO_NUM):
            self.robot_list.append(Robot(id=i))

    def init_handles(self, num):
        if num == 18:
            self.choose = 2
        if num == 50:
            self.choose = 3
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
            r.is_colide = False
            r.is_avoid = False
            r.strategy()
        #用人工势场方法进行防撞修正
        for i in range(ROBO_NUM):
            r: Robot = self.robot_list[i]
            #计算合力
            robot_force_x = 0
            robot_force_y = 0
            #计算到目标点的引力
            aforce_x, aforce_y = r.attractive_force()
            #计算到其他机器人的斥力
            for j in range(ROBO_NUM):
                if i == j:
                    continue
                r_: Robot = self.robot_list[j]
                distance = get_distance(r.x, r_.x, r.y, r_.y)
                if distance < 3 * ROBO_RADIUS_FULL * (r.get_speed()) / 3 + 3:
                    #计算到其他机器人的斥力
                    dx = r.x - r_.x
                    dy = r.y - r_.y
                    robot_force_x += 10 * ROBO_RADIUS_FULL * dx / distance**3
                    robot_force_y += 10 * ROBO_RADIUS_FULL * dy / distance**3
                if distance < 2 * ROBO_RADIUS_FULL:
                    r.is_colide = True
            #计算合力向量，并根据现在机器人速度和合力夹角给出调整
            # print_to_txt(
            #     str(self.frame) + " id:" + str(r.id) + " force:" +
            #     str(robot_force_x) + " " + str(robot_force_y))
            force = np.array(
                [robot_force_x + aforce_x, robot_force_y + aforce_y])
            dir1 = np.array([np.cos(r.direction), np.sin(r.direction)])
            # print_to_txt("dir " + str(dir1) + " " + str(r.direction))
            F = np.linalg.norm(force)  #合力绝对值
            if F > 2:
                cos_angle = np.dot(force, dir1) / F
                angle = np.arccos(cos_angle)
                cross = -np.cross(force, dir1)
                # print_to_txt("angle:" + str(angle) + " " + str(cross))
                if r.is_colide:
                    r.is_avoid = False
                    if cross > 0:
                        r.set_rotate(np.pi)
                    else:
                        r.set_rotate(-np.pi)
                    continue
                if angle > np.pi * 0.7:
                    if F > 4:
                        if r.object_type:
                            r.is_avoid = False
                            r.set_speed(r.get_speed() - F / 3)
                        else:
                            r.set_speed(r.get_speed() - F / 5)
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + F / np.pi)
                    if cross <= 0:
                        r.set_rotate(r.rotate_speed - F / np.pi)
                elif angle < np.pi * 0.25:
                    if F > 3:
                        if r.object_type:
                            r.is_avoid = False
                            r.set_speed(r.get_speed() + F / 3)
                        else:
                            r.set_speed(r.get_speed() + F / 4)
                else:
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + F / np.pi)
                    if cross <= 0:
                        r.set_rotate(r.rotate_speed - F / np.pi)
            # elif np.linalg.norm(force) > 5:
            #     if r.get_speed() < 4:
            #         r.set_speed(r.get_speed() + 1)
            # print_to_txt("set " + str(r.id) + " -1")

        for r in self.robot_list:
            #如果靠近边缘则限制最大速度
            if is_boundary(r.x, r.y, r.direction) and r.get_speed() > 3:
                r.set_speed(3)
            output = self.strategy_to_str(r)
            if output == "":
                continue
            outputs += output
        return outputs

    def output_strategy2(self):
        outputs = ""
        for r in self.robot_list:
            r.is_avoid = False
            r.strategy()
        #用人工势场方法进行防撞修正
        for i in range(ROBO_NUM):
            r: Robot = self.robot_list[i]
            #计算合力
            robot_force_x = 0
            robot_force_y = 0
            #计算到目标点的引力
            aforce_x, aforce_y = r.attractive_force2()
            #计算到其他机器人的斥力
            for j in range(ROBO_NUM):
                if i != j:
                    r_: Robot = self.robot_list[j]
                    distance = get_distance(r.x, r_.x, r.y, r_.y)
                    if distance < 1.8 * ROBO_RADIUS_FULL * r.get_speed(
                    ) / 6 + 2.5:
                        #计算到其他机器人的斥力
                        dx = r.x - r_.x
                        dy = r.y - r_.y
                        robot_force_x += dx / distance * (
                            24 * ROBO_RADIUS_FULL / distance -
                            5 * ROBO_RADIUS_FULL)
                        robot_force_y += dy / distance * (
                            24 * ROBO_RADIUS_FULL / distance -
                            5 * ROBO_RADIUS_FULL)
            #计算边界对机器人的斥力
            # Boundaty = 3
            # if r.x < Boundaty:
            #     robot_force_x += 0.8
            # if r.x > MAP_SIZE - Boundaty:
            #     robot_force_x -= 0.8
            # if r.y < Boundaty:
            #     robot_force_y += 0.8
            # if r.y > MAP_SIZE - Boundaty:
            #     robot_force_y -= 0.8
            #计算合力向量，并根据现在机器人速度和合力夹角给出调整
            # print_to_txt(
            #     str(self.frame) + " id:" + str(r.id) + " force:" +
            #     str(robot_force_x) + " " + str(robot_force_y))
            force = np.array(
                [robot_force_x + aforce_x, robot_force_y + aforce_y])
            dir1 = np.array([np.cos(r.direction), np.sin(r.direction)])
            # print_to_txt("dir " + str(dir1) + " " + str(r.direction))
            force_val = np.linalg.norm(force)
            if force_val > 0.9:
                cos_angle = np.dot(force, dir1) / force_val
                angle = np.arccos(cos_angle)
                cross = -np.cross(force, dir1)
                # print_to_txt("angle:" + str(angle) + " " + str(cross))
                force
                if angle > np.pi * 0.8:
                    if force_val > 3:
                        if r.object_type:
                            r.set_speed(r.get_speed() - force_val / 20)
                        else:
                            r.set_speed(r.get_speed() - force_val / 30)
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + force_val * angle / 30)
                    # print_to_txt("set " + str(r.id) + " 1")
                    if cross <= 0:
                        r.set_rotate(r.rotate_speed - force_val * angle / 30)
                elif angle < np.pi * 0.2:
                    if force_val > 3:
                        r.set_speed(r.get_speed() + force_val / 18)
                else:
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + cross / 20)
                    # print_to_txt("set " + str(r.id) + " 1")
                    if cross < 0:
                        r.set_rotate(r.rotate_speed + cross / 20)
            # elif np.linalg.norm(force) > 5:
            #     if r.get_speed() < 4:
            #         r.set_speed(r.get_speed() + 1)
            # print_to_txt("set " + str(r.id) + " -1")

        for r in self.robot_list:
            # 如果靠近边缘则限制最大速度
            if is_boundary(r.x, r.y, r.direction) and r.get_speed() > 3:
                r.set_speed(3)
            output = self.strategy_to_str(r)
            if output == "":
                continue
            outputs += output
        return outputs

    def output_strategy3(self):
        outputs = ""
        for r in self.robot_list:
            r.is_avoid = False
            r.strategy()
        #用人工势场方法进行防撞修正
        for i in range(ROBO_NUM):
            r: Robot = self.robot_list[i]
            #计算合力
            robot_force_x = 0
            robot_force_y = 0
            #计算到目标点的引力
            aforce_x, aforce_y = r.attractive_force2()
            #计算到其他机器人的斥力
            for j in range(ROBO_NUM):
                if i != j:
                    r_: Robot = self.robot_list[j]
                    distance = get_distance(r.x, r_.x, r.y, r_.y)
                    if distance < 1.8 * ROBO_RADIUS_FULL * r.get_speed(
                    ) / 6 + 2.5:
                        #计算到其他机器人的斥力
                        dx = r.x - r_.x
                        dy = r.y - r_.y
                        robot_force_x += dx / distance * (
                            24 * ROBO_RADIUS_FULL / distance -
                            5 * ROBO_RADIUS_FULL)
                        robot_force_y += dy / distance * (
                            24 * ROBO_RADIUS_FULL / distance -
                            5 * ROBO_RADIUS_FULL)
            #计算边界对机器人的斥力
            # Boundaty = 3
            # if r.x < Boundaty:
            #     robot_force_x += 0.8
            # if r.x > MAP_SIZE - Boundaty:
            #     robot_force_x -= 0.8
            # if r.y < Boundaty:
            #     robot_force_y += 0.8
            # if r.y > MAP_SIZE - Boundaty:
            #     robot_force_y -= 0.8
            #计算合力向量，并根据现在机器人速度和合力夹角给出调整
            # print_to_txt(
            #     str(self.frame) + " id:" + str(r.id) + " force:" +
            #     str(robot_force_x) + " " + str(robot_force_y))
            force = np.array(
                [robot_force_x + aforce_x, robot_force_y + aforce_y])
            dir1 = np.array([np.cos(r.direction), np.sin(r.direction)])
            # print_to_txt("dir " + str(dir1) + " " + str(r.direction))
            force_val = np.linalg.norm(force)
            if force_val > 1.1:
                cos_angle = np.dot(force, dir1) / force_val
                angle = np.arccos(cos_angle)
                cross = -np.cross(force, dir1)
                # print_to_txt("angle:" + str(angle) + " " + str(cross))
                force
                if angle > np.pi * 0.8:
                    if force_val > 3:
                        if r.object_type:
                            r.set_speed(r.get_speed() - force_val / 20)
                        else:
                            r.set_speed(r.get_speed() - force_val / 30)
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + force_val * angle / 30)
                    # print_to_txt("set " + str(r.id) + " 1")
                    if cross <= 0:
                        r.set_rotate(r.rotate_speed - force_val * angle / 30)
                elif angle < np.pi * 0.3:
                    if force_val > 3:
                        r.set_speed(r.get_speed() + force_val / 18)
                else:
                    if cross > 0:
                        r.set_rotate(r.rotate_speed + cross / 20)
                    # print_to_txt("set " + str(r.id) + " 1")
                    if cross < 0:
                        r.set_rotate(r.rotate_speed + cross / 20)
            # elif np.linalg.norm(force) > 5:
            #     if r.get_speed() < 4:
            #         r.set_speed(r.get_speed() + 1)
            # print_to_txt("set " + str(r.id) + " -1")

        for r in self.robot_list:
            # 如果靠近边缘则限制最大速度
            if is_boundary(r.x, r.y, r.direction) and r.get_speed() > 3:
                r.set_speed(3)
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
            return - STORE_COST[h_.handle_type-1] * (pickup_time+delivery_time) / len(h_.material_shortage)

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
            else:
                feasible_delivery_edges.pop(h)
        

        left_frame = TOTAL_TIME * 60 * FPS - self.frame
        left_max_distance = left_frame / FPS * MAX_FORE_SPEED
        for r in self.robot_list:
            r: Robot
            if r.is_assigned_task == 0:
                if r.object_type == 0:
                    if len(feasible_pickup_edges[r]) <= 0:
                        continue
                    utility = -np.inf
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
                        r.add_task(r.x, r.y, GOTO, self.frame)
                        if left_max_distance <= MAP_SIZE * 2:
                        # print_to_txt((feasible_pickup_edges[r], feasible_delivery_edges))
                            if r.id == 0: r.add_task(MAP_SIZE, MAP_SIZE, GOTO, self.frame)
                            if r.id == 1: r.add_task(0, MAP_SIZE, GOTO, self.frame)
                            if r.id == 2: r.add_task(MAP_SIZE, 0, GOTO, self.frame)
                            if r.id == 3: r.add_task(0, 0, GOTO, self.frame)
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
