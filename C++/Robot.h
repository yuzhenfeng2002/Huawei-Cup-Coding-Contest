#pragma once
using namespace std;
class Robot
{
private:
    int id;
    double delta_time;
    Handle handle;
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
    vector<tuple<double, double, int>> task_list;
    map<int, double> strategy_dict;
    int last_assigned_time;

public:
    Robot(int id);
    void update(Handle *handle, int object_type, double time_coeff, double crash_coeff, double rotate_speed, double speed_x, double speed_y, double direction, double x, double y);
    void add_task(float _x, float _y, int todo_type, int frame);
    void strategy();
    int arrive();
};
