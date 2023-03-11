TOTAL_TIME = 3
MAP_SIZE = 50
ROBO_NUM = 4
FPS = 50
INIT_MONEY = 200000
ROBO_HANDLE_DIST = 0.4
ROBO_RADIUS_NORM = 0.45
ROBO_RADIUS_FULL = 0.53
ROBO_DENSITY = 20
HANDLE_TYPE_NUM = 9
HANDLE_OBJECT_NUM = 7
MAX_FORE_SPEED = 6
MIN_BACK_SPEED = 2
MAX_ROTATE_SPEED = 3.1415926
MAX_TRACTION = 250
MAX_MOMENT = 50

TYPE_MATERIAL = {
    1: [],
    2: [],
    3: [],
    4: [1,2],
    5: [1,3],
    6: [2,3],
    7: [4,5,6],
    8: [7],
    9: [i for i in range(1,8)]
}

MATERIAL_TYPE = {
    1: [4,5,9],
    2: [4,6,9],
    3: [5,6,9],
    4: [7,9],
    5: [7,9],
    6: [7,9],
    7: [8,9],
}