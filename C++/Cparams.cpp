#include <iostream>
#include <vector>
#include <map>
#include <cmath>
const int RAND_SEED = 42;

const int TOTAL_TIME = 3;
const int MAP_SIZE = 50;
const int ROBO_NUM = 4;
const int FPS = 50;
const int INIT_MONEY = 200000;

const double ROBO_HANDLE_DIST = 0.4;
const double ROBO_RADIUS_NORM = 0.45;
const double ROBO_RADIUS_FULL = 0.53;
const int ROBO_DENSITY = 20;
const int HANDLE_TYPE_NUM = 9;
const int HANDLE_OBJECT_NUM = 7;
int MAX_FORE_SPEED = 6;
const int MIN_BACK_SPEED = 2;
const double MAX_ROTATE_SPEED = 3.1415926;
const int MAX_TRACTION = 250;
const int MAX_MOMENT = 50;

const int FORWARD = 0;
const int ROTATE = 1;
const int BUY = 2;
const int SELL = 3;
const int DESTROY = 4;
const int GOTO = 5;
int M_PI = 3.1415926;
const double BIG_M = 1e6;
std::vector<double> BUY_PRICE = {30, 44, 58, 154, 172, 192, 760, 0, 0};
std::vector<double> SELL_PRICE = {60, 76, 92, 225, 250, 275, 1050, BIG_M, BIG_M};
std::vector<double> PRODUCE_TIME = {50, 50, 50, 500, 500, 500, 1000, 1, 1};
std::vector<double> STORE_COST(HANDLE_TYPE_NUM, 0);

void calculate_store_cost()
{
    for (int i = 0; i < HANDLE_OBJECT_NUM; i++)
    {
        for (int j = i + 1; j < HANDLE_OBJECT_NUM; j++)
        {
            int idx = i * HANDLE_OBJECT_NUM + j - (i + 1) * (i + 2) / 2;
            STORE_COST[i] += (SELL_PRICE[i] - BUY_PRICE[i] + SELL_PRICE[j] - BUY_PRICE[j]) / PRODUCE_TIME[idx];
        }
    }
    STORE_COST[3] += STORE_COST[0] + STORE_COST[1];
    STORE_COST[4] += STORE_COST[0] + STORE_COST[2];
    STORE_COST[5] += STORE_COST[1] + STORE_COST[2];
    STORE_COST[6] += STORE_COST[3] + STORE_COST[4] + STORE_COST[5];
}
double get_distance(double x1, double x2, double y1, double y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// 计算两点间角度
double get_theta(double x1, double x2, double y1, double y2)
{
    double tan, theta;
    if (x2 - x1 > 0)
    {
        tan = (y2 - y1) / (x2 - x1);
        theta = atan(tan);
    }
    else if (x2 - x1 == 0)
    {
        theta = copysign(M_PI / 2, y2 - y1);
    }
    else
    {
        tan = (y2 - y1) / (x2 - x1);
        if (y2 - y1 > 0)
        {
            theta = M_PI + atan(tan);
        }
        else
        {
            theta = -M_PI + atan(tan);
        }
    }
    return theta;
}

// 计算需要旋转的角度
double get_angle(double _theta, double theta)
{
    double angle_diff = theta - _theta;
    double angle;
    if (abs(angle_diff) < M_PI / 180 || abs(angle_diff - 2 * M_PI) < M_PI / 180)
    {
        angle = 0;
    }
    else if (abs(angle_diff) > M_PI)
    {
        if (angle_diff > 0)
        {
            angle = 2 * M_PI - abs(angle_diff);
        }
        else
        {
            angle = abs(angle_diff) - 2 * M_PI;
        }
    }
    else if (abs(angle_diff) <= M_PI)
    {
        if (angle_diff > 0)
        {
            angle = -abs(angle_diff);
        }
        else
        {
            angle = abs(angle_diff);
        }
    }
    return -angle;
}
//判断是否同向
bool is_syntropy(double direction1 ,double direction2){
    if(direction1 < direction2){
        swap(direction1, direction2);
    }
    if(direction1 - direction2 < M_PI * 0.15){
        return true;
    }else if(direction1> M_PI *0.8 && direction2<-1*M_PI*0.8){
        return true;
    }
    return false;
}

//判断是否反向
bool is_opposite(double direction1, double direction2){
    if(direction1 < direction2){
        swap(direction1, direction2);
    }
    if(abs(direction1 - direction2 - M_PI) < M_PI / 7){
        return true;
    }
    return false;
}

bool is_boundary(double x, double y, double direction){
    if(x<2 && (direction > M_PI *0.8 || direction <-1*M_PI *0.8)){
        return true;
    }
    if(x>48&&direction<M_PI*0.2 && direction>-1 * M_PI*0.2){
        return true;
    }
    if(y<2 && (direction>M_PI*0.7 && direction<M_PI*0.3)){
        return true;
    }
    if(y>48 && direction<M_PI *0.7 && direction<M_PI*0.3){
        return true;
    }
    return false;
}
std::map<int, std::vector<int>> TYPE_MATERIAL = {
    {1, {}},
    {2, {}},
    {3, {}},
    {4, {1, 2}},
    {5, {1, 3}},
    {6, {2, 3}},
    {7, {4, 5, 6}},
    {8, {7}},
    {9, {1, 2, 3, 4, 5, 6, 7}}};

std::map<int, std::vector<int>> MATERIAL_TYPE = {
    {1, {4, 5, 9}},
    {2, {4, 6, 9}},
    {3, {5, 6, 9}},
    {4, {7, 9}},
    {5, {7, 9}},
    {6, {7, 9}},
    {7, {8, 9}},
};
