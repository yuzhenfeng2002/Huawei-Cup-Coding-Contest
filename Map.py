from Params import *
import numpy as np
import random
random.seed(RAND_SEED)

def print_to_txt(string: str):
    pass
    # with open("./log.txt", "a") as f:
    #     f.write(string + '\n')

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
    if abs(angle_diff) < np.pi/180 or abs(angle_diff - 2*np.pi) < np.pi/180:
        angle = 0
    elif abs(angle_diff) > np.pi:
        if angle_diff > 0:
            angle = 2*np.pi - abs(angle_diff)
        else:
            angle = abs(angle_diff) - 2*np.pi
    elif abs(angle_diff) <= np.pi:
        if angle_diff > 0:
            angle = - abs(angle_diff)
        else:
            angle = abs(angle_diff)
    return -angle

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
            if int(material_info[HANDLE_OBJECT_NUM-m]) + self.material_onroute[m-1] == 0:
                self.material_shortage.append(m)

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
    
    def update(self, handle, object_type, time_coeff, crash_coeff, rotate_speed, speed_x, speed_y, direction, x, y):
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
            self.task_list.append((x,y,todo_type))
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
            self.task_list.append((x,y,todo_type))
    
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
        
        speed = min(distance/self.delta_time, MAX_FORE_SPEED)
        if abs(angle) > np.pi/2 : speed = 0
        if abs(angle) > np.pi/4 and distance < 4 * ROBO_HANDLE_DIST : speed = 0
        rotate_speed = min(abs(angle)/self.delta_time, MAX_ROTATE_SPEED)
        self.strategy_dict = {FORWARD: speed, ROTATE: np.sign(angle) * rotate_speed}
    
    def arrive(self):
        if self.todo_type == BUY:
            self.handle.is_assigned_pickup = 0
        elif self.todo_type == SELL:
            self.handle.material_onroute[self.object_type-1] -= 1

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
    
    def update_robot(self, id, handle_id, object_type, time_coeff, crash_coeff, rotate_speed, speed_x, speed_y, direction, x, y):
        try:
            h = None
            if handle_id != -1:
                h = self.handle_list[handle_id]
            self.robot_list[id].update(h, object_type, time_coeff, crash_coeff, rotate_speed, speed_x, speed_y, direction, x, y)
        except:
            print_to_txt(str((len(self.robot_list), id)))
    def update_handle(self, id, handle_type, x, y, left_time, material, object):
        try:
            self.handle_list[id].update(handle_type, x, y, left_time, material, object)
        except:
            print_to_txt(str((len(self.handle_list), id)))
    
    def update_handle_type_dict_first(self):
        for i in range(HANDLE_TYPE_NUM):
            self.handle_type_dict[i+1] = []
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
        for i in range(ROBO_NUM):
            r: Robot = self.robot_list[i]
            for j in range(i, ROBO_NUM):
                r_: Robot = self.robot_list[j]
                if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 10 and 
                    abs(abs(r.direction - r_.direction)-np.pi) < np.pi/6):
                    theta = get_theta(r.x, r_.x, r.y, r_.y)
                    angle = get_angle(r.direction, theta)
                    rotate_speed = -np.sign(angle) * np.pi
                    r.strategy_dict[1] = rotate_speed
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
        delivery_origins = []
        for h in self.handle_list:
            h: Handle
            if h.object == 1 and h.is_assigned_pickup == 0:
                pickup_tasks[h.handle_type - 1].append(h)
        for h in self.handle_list:
            short_material = h.material_shortage
            if len(short_material) > 0:
                delivery_tasks[h.id] = []
                delivery_tasks_h = delivery_tasks[h.id]
                avg_revenue = SELL_PRICE / len(short_material)
                for m in short_material:
                    delivery_tasks_h.append(m)
                    for h_ in pickup_tasks[m-1]:
                        h_: Handle
                        delivery_edges[(h_, h)] = (m, avg_revenue, get_distance(h_.x, h.x, h_.y, h.y))
                        delivery_origins.append(h_)

        for r in self.robot_list:
            r: Robot
            if r.is_assigned_task == 0:
                if r.object_type == 0:
                    if len(delivery_origins) <= 0:
                        continue
                    left_frame = TOTAL_TIME * 60 * FPS - self.frame
                    left_max_distance = left_frame / FPS * MAX_FORE_SPEED
                    if left_max_distance < MAP_SIZE * 2:
                        r.add_task(MAP_SIZE, MAP_SIZE, GOTO, self.frame)
                        continue
                    distance_list = [get_distance(r.x, h_.x, r.y, h_.y) for h_ in delivery_origins]
                    h_idx = np.argmin(distance_list)
                    h = delivery_origins[h_idx]
                    r.add_task(h.x, h.y, 2, self.frame)
                    h.is_assigned_pickup = 1
                    delivery_origins.pop(h_idx)
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
                    for t in random.sample(MATERIAL_TYPE[r.object_type], len(MATERIAL_TYPE[r.object_type])):
                        for h in random.sample(self.handle_type_dict[t], len(self.handle_type_dict[t])):
                            h: Handle
                            if r.object_type in h.material_shortage:
                                r.add_task(h.x, h.y, 3, self.frame)                                
                                h.material_shortage.remove(r.object_type)
                                h.material_onroute[r.object_type-1] += 1
                                break
                        if r.is_assigned_task == 1:
                            break
                    if r.is_assigned_task == 0:
                        r.add_task(r.x, r.y, 4, self.frame)
                        print_to_txt("no where to go")
