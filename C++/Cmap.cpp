#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include "Cparams.cpp"

#define M_PI 3.1415926
const double RAND_SEED = 42; // Set the value of RAND_SEED here
// const double M_PI = 3.1415926;
// print_to_txt(string: str): 这个函数定义了一个空函数，似乎是为了将一些信息打印到指定的文本文件中。
// get_distance(x1, x2, y1, y2): 这个函数用于计算两点之间的欧几里得距离，输入参数是两个点的坐标。
// get_theta(x1, x2, y1, y2): 这个函数用于计算两点之间的极角（弧度制），输入参数是两个点的坐标。
// get_angle(_theta, theta): 这个函数用于计算角度差，即第二个角度减去第一个角度后的差值，其中输入参数是两个弧度制的角度。如果角度差值在一定范围内，则将其视为0，否则按照正负值返回差值的绝对值。
void print_to_txt(std::string info) {
	// Implement the print_to_txt function here
	std::fstream f;
	f.open("./log.txt", std::ios::out);
	f << info << std::endl;
	f.close();
}
// 计算两点间距离
double get_distance(double x1, double x2, double y1, double y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
// 计算两点间角度
double get_theta(double x1, double x2, double y1, double y2) {
	double tan, theta;
	if (x2 - x1 > 0) {
		tan = (y2 - y1) / (x2 - x1);
		theta = atan(tan);
	}
	else if (x2 - x1 == 0) {
		theta = copysign(M_PI / 2, y2 - y1);
	}
	else {
		tan = (y2 - y1) / (x2 - x1);
		if (y2 - y1 > 0) {
			theta = M_PI + atan(tan);
		}
		else {
			theta = -M_PI + atan(tan);
		}
	}
	return theta;
}
// 计算需要旋转的角度
double get_angle(double _theta, double theta) {
	double angle_diff = theta - _theta;
	double angle;
	if (abs(angle_diff) < M_PI / 180 || abs(angle_diff - 2 * M_PI) < M_PI / 180) {
		angle = 0;
	}
	else if (abs(angle_diff) > M_PI) {
		if (angle_diff > 0) {
			angle = 2 * M_PI - abs(angle_diff);
		}
		else {
			angle = abs(angle_diff) - 2 * M_PI;
		}
	}
	else if (abs(angle_diff) <= M_PI) {
		if (angle_diff > 0) {
			angle = -abs(angle_diff);
		}
		else {
			angle = abs(angle_diff);
		}
	}
	return -angle;
}
// 这是一个名为Handle的类，它代表了一个可操作的物体，包含一些属性和方法：
// 属性：
// id: 物体的唯一标识符
// handle_type: 物体的类型
// x, y: 物体的坐标
// left_time: 物体离开的剩余时间
// material: 物体上的物料
// object: 物体上的对象
// is_assigned_pickup: 物体是否已被指派
// material_onroute: 物体正在路上的物料
// 方法：
// update: 更新物体的属性
// identify_short_material: 识别缺少的物料
// 在update方法中，给定物体的类型、坐标、离开时间、物料和对象，更新物体的属性。
// 在identify_short_material方法中，将物体上缺少的物料识别出来。
class Handle {
private:
public:
	Handle();
	Handle(int id): id(id), handle_type(0), x(0), y(0), left_time(-1), material(0), object(0), is_assigned_pickup(0), material_onroute(std::vector<int>(HANDLE_TYPE_NUM, 0)) {}
	void update(int handle_type, int x, int y, int left_time, int material, int object) {
		this->handle_type = handle_type;
		this->x = x;
		this->y = y;
		this->left_time = left_time;
		this->material = material;
		this->object = object;
		identify_short_material();
	}
	void identify_short_material() {
		std::bitset<HANDLE_OBJECT_NUM> material_info(this->material);
		this->material_shortage.clear();
		for (int i = 0; i < 3; ++i) {
			int m = TYPE_MATERIAL[this->handle_type][i];
			if (material_info[HANDLE_OBJECT_NUM - m] + this->material_onroute[m - 1] == 0) {
				this->material_shortage.push_back(m);
			}
		}
	}
	// int get_id() const { return id; }
	// int get_handle_type() const { return handle_type; }
	// int get_x() const { return x; }
	// int get_y() const { return y; }
	// int get_left_time() const { return left_time; }
	// int get_material() const { return material; }
	// int get_object() const { return object; }
	// bool get_is_assigned_pickup() const { return is_assigned_pickup; }
	// std::vector<int> get_material_onroute() const { return material_onroute; }
	// std::vector<int> get_material_shortage() const { return material_shortage; }
	int id;
	int handle_type;
	int x;
	int y;
	int left_time;
	int material;
	int object;
	bool is_assigned_pickup;
	std::vector<int> material_onroute;
	std::vector<int> material_shortage;
};
// # 该类包含以下属性：
// # id：机器人的标识符。
// # delta_time：更新之间的时间差，根据一个名为FPS的常量计算得出。
// # handle：对机器人的引用。
// # object_type：机器人物品类型的标识符。
// # time_coeff：时间系数。
// # crash_coeff：碰撞系数。
// # rotate_speed：机器人旋转速度。
// # speed_x：机器人沿x轴移动的速度。
// # speed_y：机器人沿y轴移动的速度。
// # direction：机器人面朝的方向。
// # x：机器人当前位置的x坐标。
// # y：机器人当前位置的y坐标。
// # is_assigned_task：标志机器人是否有任务分配。
// # x_：任务的x坐标。
// # y_：任务的y坐标。
// # task_list：机器人要完成的任务列表。
// # strategy_dict：机器人的策略字典。
// # last_assigned_time：机器人最后一次接收到任务的时间。
// # 该类还包含以下方法：
// # __init__：初始化机器人的属性。
// # update：更新机器人的属性。
// # add_task：向机器人的任务列表中添加一个新的任务。
// # strategy：确定机器人的策略。
// # arrive：机器人到达任务位置时执行的操作。
class Robot {
private:
public:
	int id;
	double delta_time;
	Handle* handle;
	int object_type;
	double time_coeff;
	double crash_coeff;
	double rotate_speed;
	double speed_x;
	double speed_y;
	double direction;
	double x;
	double y;
	int is_assigned_task;
	double x_;
	double y_;
	int todo_type;
	std::vector<std::tuple<double, double, int>> task_list;
	std::map<int, double> strategy_dict;
	int last_assigned_time;
	Robot(int id) {
		this->id = id;
		this->delta_time = 1 / FPS;
		this->handle = nullptr;
		this->object_type = 0;
		this->time_coeff = 0;
		this->crash_coeff = 0;
		this->rotate_speed = 0;
		this->speed_x = 0;
		this->speed_y = 0;
		this->direction = 0;
		this->x = 0;
		this->y = 0;
		this->is_assigned_task = 0;
		this->x_ = NULL;
		this->y_ = NULL;
		this->todo_type = 0;
		this->last_assigned_time = 0;
	}
	void update(Handle* handle, int object_type, double time_coeff, double crash_coeff, double rotate_speed, double speed_x, double speed_y, double direction, double x, double y) {
		this->handle = handle;
		this->object_type = object_type;
		this->time_coeff = time_coeff;
		this->crash_coeff = crash_coeff;
		this->rotate_speed = rotate_speed;
		this->speed_x = speed_x;
		this->speed_y = speed_y;
		this->direction = direction;
		this->x = x;
		this->y = y;
	}
	void add_task(float _x, float _y, int todo_type, int frame) {
		if (todo_type == BUY || todo_type == SELL) {
			is_assigned_task = 1;
			last_assigned_time = frame;
			if (task_list.size() == 0) {
				x_ = _x;
				y_ = _y;
				todo_type = todo_type;
			}
			task_list.push_back(std::make_tuple(_x, _y, todo_type));
		}
		else if (todo_type == DESTROY) {
			is_assigned_task = 0;
			x_ = _x;
			y_ = _y;
			todo_type = todo_type;
			task_list.insert(task_list.begin(), std::make_tuple(_x, _y, DESTROY));
		}
		else if (todo_type == GOTO) {
			is_assigned_task = 0;
			last_assigned_time = frame;
			if (task_list.size() == 0) {
				x_ = _x;
				y_ = _y;
				todo_type = todo_type;
			}
			task_list.push_back(std::make_tuple(_x, _y, todo_type));
		}
	}
	void strategy() {
		// TODO
	}
	void arrive() {
		// TODO
	}
};
// # __init__(self): 类的初始化函数，设置了一些基本的地图属性，如当前帧数、初始金钱、机器人列表、抓取器列表、抓取器类型字典等。
// # update_map(self, frame, money): 更新地图的帧数和金钱。
// # init_robots(self): 初始化机器人列表。
// # init_handles(self, num): 初始化抓取器列表。
// # update_robot(self, id, handle_id, object_type, time_coeff, crash_coeff, rotate_speed, speed_x, speed_y, direction, x, y): 更新机器人的状态。
// # update_handle(self, id, handle_type, x, y, left_time, material, object): 更新抓取器的状态。
// # update_handle_type_dict_first(self): 根据抓取器类型将抓取器列表分类。
// # strategy_to_str(self, robot: Robot): 将机器人的策略转化为字符串格式。
// # output_strategy(self): 输出机器人的策略。
// # get_short_material(self): 获取缺货的原材料列表。
// # set_robots_targets(self): 设置机器人的任务和目标位置。
// # 其中，类变量包括：
// # frame: 当前帧数。
// # money: 当前金钱。
// # robot_list: 机器人列表。
// # handle_list: 抓取器列表。
// # handle_type_dict: 抓取器类型字典。
class Map {
private:
public:
	int frame;
	int money;
	std::vector<Robot> robot_list = {};
	std::vector<Handle> handle_list = {};
	std::tuple<int, std::vector<Handle>> handle_type_dict = {};
	void update_map(int frame, int money) {
		this->frame = frame;
		this->money = money;
	}
	void init_robots() {
		for (int i = 0; i < ROBO_NUM; i++) {
			robot_list.emplace_back(Robot(i));
		}
	}
	void init_handles(int num) {
		for (int i = 0; i < num; i++) {
			handle_list.emplace_back(Handle(i));
		}
	}
	void update_robot(int id, int handle_id, int object_type,
		double time_coeff, double crash_coeff, double rotate_speed,
		double speed_x, double speed_y, double direction, double x, double y) {
		try {
			Handle* h;
			if (handle_id != -1) {
				h = &handle_list[handle_id];
			}
			robot_list[id].handle = h;
			robot_list[id].object_type = object_type;
			robot_list[id].time_coeff = time_coeff;
			robot_list[id].crash_coeff = crash_coeff;
			robot_list[id].rotate_speed = rotate_speed;
			robot_list[id].speed_x = speed_x;
			robot_list[id].speed_y = speed_y;
			robot_list[id].direction = direction;
			robot_list[id].x = x;
			robot_list[id].y = y;
		}
		catch (...) {
			std::string info = "size of robot_lsit: " + std::to_string(robot_list.size()) + "id: " + std::to_string(id);
			print_to_txt(info);
		}
	}
	void update_handle(int id, int handle_type, int x, int y, int left_time,
		int material, int object) {
		try {
			handle_list[id].handle_type = handle_type;
			handle_list[id].x = x;
			handle_list[id].y = y;
			handle_list[id].left_time = left_time;
			handle_list[id].material = material;
			handle_list[id].object = object;
		}
		catch(...){
			std::string info = "size of robot_list: " + std::to_string(handle_type) + "id: "+ std::to_string(id);
		}
	}

	void update_handle_type_dict_first(){}

	void strategy_to_str(Robot robot){}

	void output_strategy(){}

	void get_short_material(){}

	void set_robots_targets(){}
};