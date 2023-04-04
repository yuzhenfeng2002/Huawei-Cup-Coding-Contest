import math
import time
import numpy as np
import matplotlib.pyplot as plt

show_animation = True


class BidirectionalAStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x, self.min_y = None, None
        self.max_x, self.max_y = None, None
        self.x_width, self.y_width, self.obstacle_map = None, None, None
        self.resolution = resolution
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
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
        Bidirectional A star path search

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

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()
        open_set_A[self.calc_grid_index(start_node)] = start_node
        open_set_B[self.calc_grid_index(goal_node)] = goal_node

        current_A = start_node
        current_B = goal_node
        meet_point_A, meet_point_B = None, None

        while True:
            if len(open_set_A) == 0:
                # print("Open set A is empty..")
                break

            if len(open_set_B) == 0:
                # print("Open set B is empty..")
                break

            c_id_A = min(
                open_set_A,
                key=lambda o: self.find_total_cost(open_set_A, o, current_B))

            current_A = open_set_A[c_id_A]

            c_id_B = min(
                open_set_B,
                key=lambda o: self.find_total_cost(open_set_B, o, current_A))

            current_B = open_set_B[c_id_B]

            if current_A.x == current_B.x and current_A.y == current_B.y:
                # print("Found goal")
                meet_point_A = current_A
                meet_point_B = current_B
                break

            # Remove the item from the open set
            del open_set_A[c_id_A]
            del open_set_B[c_id_B]

            # Add it to the closed set
            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):

                c_nodes = [self.Node(current_A.x + self.motion[i][0],
                                     current_A.y + self.motion[i][1],
                                     current_A.cost + self.motion[i][2],
                                     c_id_A),
                           self.Node(current_B.x + self.motion[i][0],
                                     current_B.y + self.motion[i][1],
                                     current_B.cost + self.motion[i][2],
                                     c_id_B)]

                n_ids = [self.calc_grid_index(c_nodes[0]),
                         self.calc_grid_index(c_nodes[1])]

                # If the node is not safe, do nothing
                continue_ = self.check_nodes_and_sets(c_nodes, closed_set_A,
                                                      closed_set_B, n_ids)

                if not continue_[0]:
                    if n_ids[0] not in open_set_A:
                        # discovered a new node
                        open_set_A[n_ids[0]] = c_nodes[0]
                    else:
                        if open_set_A[n_ids[0]].cost > c_nodes[0].cost:
                            # This path is the best until now. record it
                            open_set_A[n_ids[0]] = c_nodes[0]

                if not continue_[1]:
                    if n_ids[1] not in open_set_B:
                        # discovered a new node
                        open_set_B[n_ids[1]] = c_nodes[1]
                    else:
                        if open_set_B[n_ids[1]].cost > c_nodes[1].cost:
                            # This path is the best until now. record it
                            open_set_B[n_ids[1]] = c_nodes[1]

        rx, ry = self.calc_final_bidirectional_path(
            meet_point_A, meet_point_B, closed_set_A, closed_set_B)
        if len(rx) != len(ry):
            return list(0, 0)
        return list(zip(rx, ry))

    # takes two sets and two meeting nodes and return the optimal path
    def calc_final_bidirectional_path(self, n1, n2, setA, setB):
        #加入找不到路径的判断
        if(n1==None or n2==None):
            return [-1], [-1]
        rx_A, ry_A = self.calc_final_path(n1, setA)
        rx_B, ry_B = self.calc_final_path(n2, setB)
        rx_A.reverse()
        ry_A.reverse()

        rx = rx_A + rx_B
        ry = ry_A + ry_B

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], \
                 [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def check_nodes_and_sets(self, c_nodes, closedSet_A, closedSet_B, n_ids):
        continue_ = [False, False]
        if not self.verify_node(c_nodes[0]) or n_ids[0] in closedSet_A:
            continue_[0] = True

        if not self.verify_node(c_nodes[1]) or n_ids[1] in closedSet_B:
            continue_[1] = True

        return continue_

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def find_total_cost(self, open_set, lambda_, n1):
        g_cost = open_set[lambda_].cost
        h_cost = self.calc_heuristic(n1, open_set[lambda_])
        f_cost = g_cost + h_cost
        return f_cost

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
        if node.x > len(self.obstacle_map)-1 or node.y > len(self.obstacle_map[0])-1:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
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
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    def get_turning_points(self,path):
        turning_points = []
        for i in range(1, len(path) - 1):
            p1 = path[i-1]
            p2 = path[i]
            p3 = path[i+1]
            angle = self.get_angle(p1, p2, p3)
            if angle != 180:
                turning_points.append(p2)
        return turning_points

    def get_angle(self, p1, p2, p3):
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        dot_product = v1[0]*v2[0] + v1[1]*v2[1]
        len1 = math.sqrt(v1[0]**2 + v1[1]**2)
        len2 = math.sqrt(v2[0]**2 + v2[1]**2)
        cos_theta = dot_product / (len1 * len2)
        angle = math.acos(cos_theta) * 180 / math.pi
        return angle

def main():
    print(__file__ + " start!!")
    map_array = []
    file_path = '../maps/3.txt'
    with open(file_path, 'r') as f:
        for line in f:
            row = [1 if c == '#' else 0 for c in line.strip()]
            map_array.append(row)
    # start and goal position
    sx = 5.0  # [m]
    sy = 5.0  # [m]
    gx = 30.0  # [m]
    gy = 25.0  # [m]
    grid_size = 1# [m]
    robot_radius = 0.7  # [m]
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
    for i in range(len(map_array)-1):
        for j in range(len(map_array)-1):
            if map_array[i][j] == 1:
                ox.append(j * 0.5-0.25)
                oy.append(-i * 0.5 - 50+0.25)
    # # # set obstacle positions
    # ox, oy = [], []
    # for i in range(-10, 60):
    #     ox.append(i)
    #     oy.append(-10.0)
    # for i in range(-10, 60):
    #     ox.append(60.0)
    #     oy.append(i)
    # for i in range(-10, 61):
    #     ox.append(i)
    #     oy.append(60.0)
    # for i in range(-10, 61):
    #     ox.append(-10.0)
    #     oy.append(i)
    # for i in range(-10, 40):
    #     ox.append(20.0)
    #     oy.append(i)
    # for i in range(0, 40):
    #     ox.append(40.0)
    #     oy.append(60.0 - i)
    print("ox:"+str(len(ox)))
    bidir_a_star = BidirectionalAStarPlanner(ox,oy,grid_size, robot_radius)
    bidir_a_star.calc_obstacle_map(ox,oy)
    # for i in range(0, 20):
    #     ox.append(40.0)
    #     oy.append(20.0 - i)
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "ob")
    plt.grid(True)
    bidir_a_star.calc_obstacle_map(ox,oy)
    start_time = time.time()
    path = bidir_a_star.planning(sx, sy, gx, gy)
    # path = bidir_a_star.get_turning_points(path)
    end_time = time.time()
    print("程序计算时间为：", end_time - start_time, "秒")
    if True:  # pragma: no cover
        plt.axis("equal")
        x_values = [coord[0] for coord in path]
        y_values = [coord[1] for coord in path]
        plt.plot(x_values, y_values, "-r")
        plt.pause(.0001)
        plt.show()


if __name__ == '__main__':
    main()
