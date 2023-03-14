#include <iostream>
#include <vector>
#include <map>

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
const int MAX_FORE_SPEED = 6;
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