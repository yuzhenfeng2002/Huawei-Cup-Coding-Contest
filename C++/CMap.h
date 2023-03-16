#pragma once
#include "Robot.h"
#include "Handle.h"
#include <vector>
#include <set>
using namespace std;
class Map
{
private:
public:
	int frame;
	int money;
	vector<Robot> robot_list;
	vector<Handle> handle_list;
	std::map<int, std::vector<Handle>> handle_type_dict = {};

	Map();
	Map(int id);
	void update_map(int frame, int money);
	void init_robots();
	void init_handles(int num);
	void update_robot(int id, int handle_id, int object_type,
                       double time_coeff, double crash_coeff,
                       double rotate_speed, double speed_x, double speed_y,
                       double direction, double x, double y);
	void update_handle(int id, int handle_type, int x, int y, int left_time,
                        int material, int object);
	
	// TODO未完成
	void Map::update_handle_type_dict_first();
	string strategy_to_str(Robot robot);
	string output_strategy();
	set<int> get_short_material();
	
	int set_robots_targets();
};
