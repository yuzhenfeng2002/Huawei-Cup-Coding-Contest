#pragma once
#include "Robot.h"
#include "Handle.h"
#include <vector>
using namespace std;
class Map
{
private:
public:
	int frame;
	int money;
	vector<Robot> robots;
	vector<Handle> Handle_list;
	vector<Handle> handle_type_dict;


	Map(int id);
	void update_map(int frame, int money);
	void init_robots();
	void init_handles(int num);
	void update_robot(Handle *handle, int object_type, double time_coeff, double crash_coeff, double rotate_speed, double speed_x, double speed_y, double direction, double x, double y);
	void update_handle(int handle_type, int x, int y, int left_time, int material, int object);
	int set_robots_targets();
	// TODO未完成
};
