#include "Robot.h"
#include "Handle.h"
Robot::Robot(int id)
{
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

void Robot::update(Handle *handle, int object_type, double time_coeff, double crash_coeff, double rotate_speed, double speed_x, double speed_y, double direction, double x, double y)
{
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

void Robot::add_task(float _x, float _y, int todo_type, int frame)
{
    if (todo_type == BUY || todo_type == SELL)
    {
        is_assigned_task = 1;
        last_assigned_time = frame;
        if (task_list.size() == 0)
        {
            x_ = _x;
            y_ = _y;
            todo_type = todo_type;
        }
        task_list.push_back(std::make_tuple(_x, _y, todo_type));
    }
    else if (todo_type == DESTROY)
    {
        is_assigned_task = 0;
        x_ = _x;
        y_ = _y;
        todo_type = todo_type;
        task_list.insert(task_list.begin(), std::make_tuple(_x, _y, DESTROY));
    }
    else if (todo_type == GOTO)
    {
        is_assigned_task = 0;
        last_assigned_time = frame;
        if (task_list.size() == 0)
        {
            x_ = _x;
            y_ = _y;
            todo_type = todo_type;
        }
        task_list.push_back(std::make_tuple(_x, _y, todo_type));
    }
}

void Robot::strategy()
{
    strategy_dict.clear();

    if (x_ == NULL || y_ == NULL)
    {
        map<int, double> stop_strategy = {{FORWARD, 0}, {ROTATE, 0}};
        strategy_dict = stop_strategy;
        return;
    }

    double distance = get_distance(x, x_, y, y_);
    if (distance < ROBO_HANDLE_DIST)
    {
        map<int, double> stop_strategy = {{FORWARD, 0}, {ROTATE, 0}, {arrive(), -1}};
        strategy_dict = stop_strategy;
        return;
    }

    float theta = get_theta(x, x_, y, y_);
    float angle = get_angle(direction, theta);
    // TODO,这里min重载有点问题
    float speed = min((distance / delta_time), MAX_FORE_SPEED);
    if (abs(angle) > M_PI / 2)
        speed = 0;
    if (abs(angle) > M_PI / 4 && distance < 4 * ROBO_HANDLE_DIST)
        speed = 0;
    float rotate_speed = min(abs(angle) / delta_time, MAX_ROTATE_SPEED);
    map<int, double> strategy = {{FORWARD, speed}, {ROTATE, signbit(angle) ? -rotate_speed : rotate_speed}};
    strategy_dict = strategy;
}

int Robot::arrive()
{
    if (todo_type == BUY)
    {
        handle.is_assigned_pickup = 0;
    }
    else if (todo_type == SELL)
    {
        handle.material_onroute[object_type - 1] -= 1;
    }

    auto pretask = task_list.front();
    task_list.erase(task_list.begin());

    if (task_list.empty())
    {
        is_assigned_task = 0;
        x_ = NULL;
        y_ = NULL;
        todo_type = 0;
    }
    else
    {
        std::vector<int> task = task_list.front();
        x_ = task[0];
        y_ = task[1];
        todo_type = task[2];
    }
    return get<2>(pretask);
}