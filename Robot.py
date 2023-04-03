import numpy as np
from Params import *
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

        if True:
            stop_strategy = {FORWARD: 0, ROTATE: 0}
            self.strategy_dict = stop_strategy
            return

    #轨迹跟踪规划
    def strategy_path(self):
        self.strategy_dict = {}
        if len(self.path) > 0:
            self.path_x, self.path_y = self.path[0]  #可优化
        if self.x_ is None or self.y_ is None or self.is_assigned_task==0:
            stop_strategy = {FORWARD: 0, ROTATE: 0}
            self.strategy_dict = stop_strategy
            return
        if len(self.path) == 0:
            self.is_plan = False
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
        if len(self.path)>1:
            if distance<ROBO_HANDLE_DIST*1.5:
                self.path.pop(0)
                if len(self.path) > 0:
                    self.path_x, self.path_y = self.path[0]  #可优化
        else:
            if distance<ROBO_HANDLE_DIST:
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
    #轨迹跟踪规划
    def strategy_path4(self):
        self.strategy_dict = {}
        if len(self.path) > 0:
            self.path_x, self.path_y = self.path[0]  #可优化
        if self.x_ is None or self.y_ is None:
            stop_strategy = {FORWARD: 0, ROTATE: 0}
            self.strategy_dict = stop_strategy
            return
        if len(self.path) == 0:
            self.is_plan = False
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
        if distance<ROBO_HANDLE_DIST:
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