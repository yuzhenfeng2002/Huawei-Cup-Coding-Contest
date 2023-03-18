#include "CMap.h"
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>

//定义结构体，用于delivery_edges作为最小型元素
struct delivery_edges_node
{
    Handle h;
    double avg_revenue;
    double distance;
};

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

string Map::strategy_to_str(Robot robot)
{
    std::string strategy_str = "";
    for (auto &p : robot.strategy_dict)
    {
        if (p.first == FORWARD)
        {
            strategy_str +=
                "forward " + to_string(robot.id) + " " + to_string(p.second);
        }
        else if (p.first == ROTATE)
        {
            strategy_str +=
                "rotate " + to_string(robot.id) + " " + to_string(p.second);
        }
        else if (p.first == BUY)
        {
            strategy_str +=
                "buy " + to_string(robot.id) + " " + to_string(p.second);
        }
        else if (p.first == SELL)
        {
            strategy_str +=
                "sell " + to_string(robot.id) + " " + to_string(p.second);
        }
        else if (p.first == DESTROY)
        {
            strategy_str +=
                "destroy " + to_string(robot.id) + " " + to_string(p.second);
        }
        return strategy_str;
    }
}

string Map::output_strategy()
{
    std::string outputs = "";
    for (Robot &r : robot_list)
    {
        r.strategy();
    }
    for (int i = 0; i < ROBO_NUM; i++)
    {
        Robot r = robot_list[i];
        for (int j = i; j < ROBO_NUM; j++)
        {
            Robot r_ = robot_list[j];
            // 靠近
            if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 10 &&
                abs(abs(r.direction - r_.direction) - M_PI) < M_PI / 6)
            {
                double theta = get_theta(r.x, r_.x, r.y, r_.y);
                double angle = get_angle(r.direction, theta);
                double rotate_speed = -1 * std::sin(angle) * M_PI;
                if (get_distance(r.x, r_.x, r.y, r_.y) < ROBO_RADIUS_FULL * 3)
                {
                    if (r_.is_avoid == false)
                    {
                        r.strategy_dict[1] = rotate_speed;
                        r.strategy_dict[0] = 2;
                        r.is_avoid = true;
                    }
                    else
                    {
                        r.strategy_dict[1] = rotate_speed * 0.6;
                        r.is_avoid = false;
                    }
                }
            }
        }
    }
    for (auto &r : robot_list)
    {
        // 如果靠近边缘则限制最大速度
        if (is_boundary(r.x, r.y, r.direction) && r.strategy_dict[0] > 4)
        {
            r.strategy_dict[0] = 4;
        }
        std::string output = strategy_to_str(r);
        if (output == "")
        {
            continue;
        }
        outputs += output;
    }
    return outputs;
}

set<int> Map::get_short_material()
{
    set<int> short_material;
    for (auto &h : handle_list)
    {
        for (auto ele : h.material_shortage)
        {
            short_material.insert(ele);
        }
    }
    return short_material;
}
int Map::set_robots_targets()
{
    vector<vector<Handle>> pickup_tasks(HANDLE_OBJECT_NUM, vector<Handle>());
    map<int, vector<int>> delivery_tasks;
    map<Handle, vector<delivery_edges_node>> delivery_edges;

    for (Handle &h : handle_list)
    {
        if (h.object == 1 && h.is_assigned_pickup == 0)
        {
            pickup_tasks[h.handle_type - 1].emplace_back(h);
        }
    }

    for (Handle &h : handle_list)
    {
        vector<int> short_material = h.material_shortage;
        if (short_material.size() > 0)
        {
            delivery_tasks.insert({h.id, vector<int>()});
            vector<int> delivery_tasks_h = delivery_tasks.at(h.id);
            double avg_revenue =
                (SELL_PRICE[h.handle_type - 1] - BUY_PRICE[h.handle_type - 1]) /
                short_material.size();
            for (auto &m : short_material)
            {
                delivery_tasks_h.emplace_back(m);
                for (Handle &h_ : pickup_tasks[m - 1])
                {
                    delivery_edges_node edges_node;
                    edges_node.h = h;
                    edges_node.avg_revenue = avg_revenue;
                    edges_node.distance = get_distance(h_.x, h.x, h_.y, h.y);
                    delivery_edges.at(h_).emplace_back(edges_node);
                }
            }
        }
    }
    vector<Handle> delivery_origins;
    for (auto &p : delivery_edges)
    {
        delivery_origins.emplace_back(p.first);
    }

    for (Robot &r : robot_list)
    {
        if (r.is_assigned_task == 0)
        {
            if (r.object_type == 0)
            {
                if (delivery_origins.size() <= 0)
                {
                    continue;
                }
                double left_frame = TOTAL_TIME * 60 * FPS - frame;
                double left_max_distance = left_frame / FPS * MAX_FORE_SPEED;
                vector<double> distance_list;
                for (Handle &h_ : delivery_origins)
                {
                    distance_list.emplace_back(
                        get_distance(r.x, h_.x, r.y, h_.y) / MAX_FORE_SPEED *
                        STORE_COST[h_.handle_type - 1]);
                }
                int h_idx =
                    min_element(distance_list.begin(), distance_list.end()) -
                    distance_list.begin();
                Handle h = delivery_origins[h_idx];

                vector<double> revenue_list;
                for (auto &edges_node : delivery_edges.at(h))
                {
                    double res_rev =
                        -1 * (edges_node.avg_revenue -
                              edges_node.distance / MAX_FORE_SPEED *
                                  STORE_COST[edges_node.h.handle_type - 1]);
                    revenue_list.emplace_back(res_rev);
                }
                sort(revenue_list.begin(), revenue_list.end());
                for (int h__idx = 0; h__idx < revenue_list.size(); h__idx++)
                {
                    Handle h_ = delivery_edges.at(h)[h__idx].h;
                    if (count(h_.material_shortage.begin(),
                              h_.material_shortage.end(), h.handle_type))
                    {
                        break;
                    }
                    // 将下面的if结构，放到for循环里面
                    if (get_distance(r.x, h.x, r.y, h.y) +
                            get_distance(h.x, h_.x, h.y, h_.y) <
                        left_max_distance * 0.8)
                    {
                        r.add_task(h.x, h.y, 2, frame);
                        h.is_assigned_pickup = 1;
                        delivery_origins.pop_back();
                        r.add_task(h_.x, h_.y, SELL, frame);
                        if (count(h_.material_shortage.begin(),
                                  h_.material_shortage.end(), h.handle_type))
                        {
                            vector<int>::iterator iter =
                                find(h_.material_shortage.begin(),
                                     h_.material_shortage.end(), h.handle_type);
                            h_.material_shortage.erase(iter);
                        }
                        h_.material_onroute[h.handle_type - 1]++;
                        delivery_edges[h].pop_back();
                    }
                    else
                    {
                        r.add_task(MAP_SIZE, MAP_SIZE, GOTO, frame);
                    }
                }
            }
            else
            {
                // 创建随机列表
                vector<int> material_type_list = MATERIAL_TYPE.at(r.object_type);
                srand(unsigned(time(NULL)));
                random_shuffle(material_type_list.begin(), material_type_list.end());
                for(int t: material_type_list){
                    vector<Handle> handle_type_dict_list = handle_type_dict.at(t);
                    random_shuffle(handle_type_dict_list.begin(), handle_type_dict_list.end());
                    for(Handle &h: handle_type_dict_list){
                        if(count(h.material_shortage.begin(),h.material_shortage.end(),r.object_type)){
                            r.add_task(h.x,h.y,3,frame);
                            vector<int>::iterator iter = find(h.material_shortage.begin(), h.material_shortage.end(),r.object_type);
                            h.material_shortage.erase(iter);
                            // 此处可能会造成内存问题
                            h.material_onroute[r.object_type-1]++;
                            break;
                        }
                    }
                    if(r.is_assigned_task ==1){
                        break;
                    }
                }
                if(r.is_assigned_task == 0){
                    r.add_task(r.x,r.y,4,frame);
                    print_to_txt("no where to go");
                }
            }
        }
    }
}