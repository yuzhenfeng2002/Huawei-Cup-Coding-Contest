#!/bin/bash
import sys

from Map import *


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    read_util_ok()
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
            m.init_handles(handle_num)
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
