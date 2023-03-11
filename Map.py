from Params import *
import numpy as np
import random

def print_to_txt(string: str):
    pass
    # with open("./log.txt", "a") as f:
    #     f.write(string + '\n')

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
        if self.object == 1:
            self.material_shortage = []
            return
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
    
    def add_task(self, x, y, todo_type):
        if todo_type in [2, 3]:
            self.is_assigned_task = 1
            if len(self.task_list) == 0:
                self.x_ = x
                self.y_ = y
                self.todo_type = todo_type
            self.task_list.append((x,y,todo_type))
        elif todo_type == 4:
            self.is_assigned_task = 0
            self.task_list.insert(0, (x, y, 4))
            self.x_ = x
            self.y_ = y
            self.todo_type = todo_type
    
    def strategy(self):
        if self.x_ is None and self.y_ is None:
            return []
        
        distance = np.sqrt((self.x_ - self.x)**2 + (self.y_ - self.y)**2)
        if distance < ROBO_HANDLE_DIST:
            return [(0, 0), (1, 0)] + self.arrive()

        if self.x_ - self.x > 0:
            tan = (self.y_ - self.y) / (self.x_ - self.x)
            theta = np.arctan(tan)
        elif self.x_ - self.x == 0:
            theta = np.sign(self.y_ - self.y) * np.pi / 2
        else:
            tan = (self.y_ - self.y) / (self.x_ - self.x)
            if self.y_ - self.y > 0:
                theta = np.pi + np.arctan(tan)
            else:
                theta = -np.pi + np.arctan(tan)
        
        angle_diff = theta - self.direction
        if abs(angle_diff) < 1e-3 or abs(angle_diff - 2*np.pi) < 1e-3:
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
        speed = min(distance/self.delta_time, MAX_FORE_SPEED)
        if abs(angle) > np.pi/2 : speed = 0
        if abs(angle) > np.pi/4 and distance < 2 * ROBO_HANDLE_DIST : speed = 0
        rotate_speed = min(abs(angle)/self.delta_time, MAX_ROTATE_SPEED)
        return [(0, speed), (1, -np.sign(angle) * rotate_speed)]
    
    def arrive(self):
        if self.todo_type == 2:
            self.handle.is_assigned_pickup = 0
        elif self.todo_type == 3:
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
        return [(pretask[2], -1)]

    def strategy_to_str(self):
        strategy_list = self.strategy()
        strategy_str = ""
        for s in strategy_list:
            if s[0] == 0:
                strategy_str += "forward {:.0f} {:}\n".format(self.id, s[1])
            elif s[0] == 1:
                strategy_str += "rotate {:.0f} {:}\n".format(self.id, s[1])
            elif s[0] == 2:
                strategy_str += "buy {:.0f}\n".format(self.id)
            elif s[0] == 3:
                strategy_str += "sell {:.0f}\n".format(self.id)
            elif s[0] == 4:
                strategy_str += "destroy {:.0f}\n".format(self.id)
        return strategy_str

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
    
    def get_short_material(self):
        short_material = set()
        for h in self.handle_list:
            h: Handle
            short_material = short_material.union(h.material_shortage)
        return short_material

    def set_robots_targets(self):
        for r in self.robot_list:
            r: Robot
            if r.is_assigned_task == 0:
                if r.object_type == 0:
                    type_list = list(self.get_short_material())
                    for t in random.sample(type_list, len(type_list)):
                        for h in random.sample(self.handle_type_dict[t], len(self.handle_type_dict[t])):
                            h: Handle
                            if h.object == 1 and h.is_assigned_pickup == 0:
                                r.add_task(h.x, h.y, 2)
                                h.is_assigned_pickup = 1
                                break
                        if r.is_assigned_task == 1:
                            break
                else:
                    for t in random.sample(MATERIAL_TYPE[r.object_type], len(MATERIAL_TYPE[r.object_type])):
                        for h in random.sample(self.handle_type_dict[t], len(self.handle_type_dict[t])):
                            h: Handle
                            if h.object == 0 and r.object_type in h.material_shortage:
                                r.add_task(h.x, h.y, 3)                                
                                h.material_shortage.remove(r.object_type)
                                h.material_onroute[r.object_type-1] += 1
                                break
                        if r.is_assigned_task == 1:
                            break
                    if r.is_assigned_task == 0:
                        r.add_task(r.x, r.y, 4)
                        print_to_txt("no where to go")
