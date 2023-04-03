#!/bin/bash
import sys

from Map import *
# import matplotlib.pyplot as plt
ox = []
oy = []

def print_to_txt(string: str, i):
    if i == 0:
        with open("./log.txt", "w") as f:
            f.write(str(string) + '\n')
    else:
        with open("./log.txt", "a") as f:
            f.write(str(string) + '\n')
#读取地图中障碍物,增加泛用性
def read_map_ok():
    i=0
    while True :
        line = input()
        if line!= "OK":
            line = [1 if c == '#' else 0 for c in line.strip()]
            j=0
            for x in line:
                if x==1:
                    ox.append(j/2+0.25)
                    oy.append(-i/2+50+0.25)
                j+=1
            i+=1
            continue
        return

def read_util_ok():
    while input()!= "OK" :
        pass
def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    read_map_ok()
    #划定边界
    for i in range(0, 100):
        ox.append(i/2)
        oy.append(0.0)
    for i in range(0, 100):
        ox.append(i/2)
        oy.append(50)
    for i in range(0, 100):
        ox.append(0.0)
        oy.append(i/2)
    for i in range(0, 100):
        ox.append(50.0)
        oy.append(i/2)

    # # 显示栅格图
    # plt.plot(ox, oy, ".k")
    # plt.show()
    finish()
    m = Map()
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame = int(parts[0])
        money = int(parts[1])
        m.update_map(frame, money)
        handle_num = int(sys.stdin.readline())
        if frame == 1:
            m.init_handles(handle_num,ox,oy)
            m.init_robots()
        for i in range(handle_num):
            line = sys.stdin.readline()
            parts = line.split(' ')
            m.update_handle(i, int(parts[0]), float(parts[1]), float(parts[2]),
                            int(parts[3]), int(parts[4]), int(parts[5]))
        for i in range(ROBO_NUM):
            line = sys.stdin.readline()
            parts = line.split(' ')
            m.update_robot(i, int(parts[0]), int(parts[1]), float(parts[2]),
                           float(parts[3]), float(parts[4]), float(parts[5]),
                           float(parts[6]), float(parts[7]), float(parts[8]),
                           float(parts[9]))
        if frame == 1:
            m.update_handle_type_dict_first()
        read_util_ok()

        m.set_robots_targets()

        sys.stdout.write('%d\n' % frame)
        if m.map_type == 3 or m.map_type == 1:
            sys.stdout.write(m.output_strategy2())
        elif m.map_type == 4:
            sys.stdout.write(m.output_strategy4())
        else:
            sys.stdout.write(m.output_strategy())
        finish()
        # if frame > 200:
        #     raise KeyError