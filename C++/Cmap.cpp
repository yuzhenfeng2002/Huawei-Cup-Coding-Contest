#include "CMap.h"
#include <fstream>
#include <vector>
#include <string>

void print_to_txt(std::string info)
{
    // Implement the print_to_txt function here
    std::fstream f;
    f.open("./log.txt", std::ios::out);
    f << info << std::endl;
    f.close();
}

void Map::update_map(int frame, int money)
{
    this->frame = frame;
    this->money = money;
}
void Map::init_robots()
{
    for (int i = 0; i < ROBO_NUM; i++)
    {
        robot_list.emplace_back(Robot(i));
    }
}
void Map::init_handles(int num)
{
    for (int i = 0; i < num; i++)
    {
        handle_list.emplace_back(Handle(i));
    }
}
void Map::update_robot(int id, int handle_id, int object_type,
                       double time_coeff, double crash_coeff,
                       double rotate_speed, double speed_x, double speed_y,
                       double direction, double x, double y)
{
    try
    {
        Handle h;
        if (handle_id != -1)
        {
            h = handle_list[handle_id];
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
    catch (...)
    {
        std::string info =
            "size of robot_lsit: " + std::to_string(robot_list.size()) +
            "id: " + std::to_string(id);
        print_to_txt(info);
    }
}
void Map::update_handle(int id, int handle_type, int x, int y, int left_time,
                        int material, int object)
{
    try
    {
        handle_list[id].handle_type = handle_type;
        handle_list[id].x = x;
        handle_list[id].y = y;
        handle_list[id].left_time = left_time;
        handle_list[id].material = material;
        handle_list[id].object = object;
    }
    catch (...)
    {
        std::string info =
            "size of robot_list: " + std::to_string(handle_type) +
            "id: " + std::to_string(id);
    }
}

void Map::update_handle_type_dict_first()
{
    for (int i = 0; i < HANDLE_TYPE_NUM; i++)
    {
        handle_type_dict[i + 1] = std::vector<Handle>();
    }
    for (Handle &h : handle_list)
    {
        handle_type_dict[h.handle_type].emplace_back(h);
    }
}

string Map::strategy_to_str(Robot robot) { 
    std::string strategy_str = ""; 
    for(auto& p: robot.strategy_dict){
        if(p.first==FORWARD){
            strategy_str += "forward " + to_string(robot.id) + " " + to_string(p.second);
        }
        else if(p.first == ROTATE){
            strategy_str += "rotate "+ to_string(robot.id) + " "+ to_string(p.second);
        }
        else if(p.first == BUY){
            strategy_str += "buy "+to_string(robot.id) + " "+to_string(p.second);
        }else if(p.first == SELL){
            strategy_str += "sell "+to_string(robot.id)+ " "+to_string(p.second);
        }else if(p.first == DESTROY){
            strategy_str += "destroy "+to_string(robot.id) + " "+to_string(p.second);
        }
        return strategy_str;
    }
    
    }

void output_strategy() {}

set<int> Map::get_short_material(){
    set<int> short_material;
    for(auto& h: handle_list){
        for(auto ele: h.material_shortage){
            short_material.insert(ele);
        }
    }
    return short_material;
}
void set_robots_targets() {}