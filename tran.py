import math
import time

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m],地图的像素
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        """定义搜索区域节点类,每个Node都包含坐标x和y, 移动代价cost和父节点索引。
        """

        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        输入起始点和目标点的坐标(sx,sy)和(gx,gy)，
        最终输出的结果是路径包含的点的坐标集合rx和ry。
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set,
                       key=lambda o: open_set[o].cost + self.calc_heuristic(
                           goal_node, open_set[o]))
            current = open_set[c_id]

            # # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect(
            #         'key_release_event',
            #         lambda event: [exit(0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)

            # 通过追踪当前位置current.x和current.y来动态展示路径寻找
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return list(zip(rx, ry))

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)
                  ], [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.insert(0, self.calc_grid_position(n.x, self.min_x))
            ry.insert(0, self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        """计算启发函数

        Args:
            n1 (_type_): _description_
            n2 (_type_): _description_

        Returns:
            _type_: _description_
        """
        # 之后可以视情况更换启发函数
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def reduce_point_by_slope(rx, ry):
        """
        reduce_point_by_slope 根据斜率经典路径点

        :param rx: AStar 返回的x路径索引
        :param ry: AStar 返回的y路径索引
        :return:
            rx_reduce:  精简过的x路径
            ry_reduce:  精简过的y路径
        """
        slope = []
        for i in range(len(rx) - 1):
            if rx[i] == rx[i + 1]:
                slope.append(math.inf)
            else:
                k = (ry[i + 1] - ry[i]) / (rx[i + 1] - rx[i])
                slope.append(k)

        ## 循环遍历斜率结点，找到斜率相互不同的点的索引
        point_ind_reduce = []
        if len(slope) == 0:
            return rx, ry
        tem_k = slope[0]
        ## 起点也被精简在外
        # print("reduce point")
        for i in range(len(slope)):
            if abs(slope[i] - tem_k) > 0.1:  ## 此处的参数可以进行调整
                # print(slope[i] - tem_k)
                point_ind_reduce.append(i)

                tem_k = slope[i]
            else:
                continue
        point_ind_reduce.append(len(slope))
        # print(len(point_ind_reduce))
        rx_reduce = []
        ry_reduce = []
        for i in range(len(point_ind_reduce)):
            rx_reduce.append(rx[point_ind_reduce[i]])
            ry_reduce.append(ry[point_ind_reduce[i]])
        return rx_reduce, ry_reduce

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1,
                                                     1], [-1, -1, 1.414],
                  [-1, 1, 1.414], [1, -1, 1.414], [1, 1, 1.414]]

        return motion


def main():
    print(__file__ + " start!!")
    map_array = []
    file_path = '../maps/2.txt'
    with open(file_path, 'r') as f:
        for line in f:
            row = [1 if c == '#' else 0 for c in line.strip()]
            map_array.append(row)
    # start and goal position
    sx = 15.75  # [m]
    sy = 26.75  # [m]
    gx = 9.25  # [m]
    gy = 29.25  # [m]
    grid_size = 0.6  # [m]
    robot_radius = 0.6  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(0, 50):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 50):
        ox.append(i)
        oy.append(50)
    for i in range(0, 50):
        ox.append(0.0)
        oy.append(i)
    for i in range(0, 50):
        ox.append(50.0)
        oy.append(i)
    for i in range(len(map_array)):
        for j in range(len(map_array)):
            if map_array[i][j] == 1:
                ox.append(j * 0.5)
                oy.append(-i * 0.5 + 50)
    # print('ox')
    # print(ox)
    # print('oy')
    # print(oy)
    # if show_animation:  # pragma: no cover
    #     plt.plot(ox, oy, ".k")
    #     plt.plot(sx, sy, "og")
    #     plt.plot(gx, gy, "xb")
    #     plt.grid(True)
    #     plt.axis("equal")
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    start_time = time.time()
    path = a_star.planning(sx, sy, gx, gy)
    end_time = time.time()
    print("程序计算时间为：", end_time - start_time, "秒")
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        # plt.plot(rx, ry, '.g')
        plt.plot(rx_reduce, ry_reduce, '.r')
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    # path.pop()
    # print('try: ' + str(path[len(path) - 1]))
    if show_animation:  # pragma: no cover
        x_values = [coord[0] for coord in path]
        y_values = [coord[1] for coord in path]
        plt.plot(x_values, y_values, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
