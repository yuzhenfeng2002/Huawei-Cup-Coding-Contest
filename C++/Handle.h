#pragma once
#include <vector>
#include <bitset>
#include "Cparams.cpp"
class Handle
{
private:
    int id;
    int handle_type;
    int x;
    int y;
    int left_time;
    int material;
    int object;

public:
    Handle();
    Handle(int id);
    void update(int handle_type, int x, int y, int left_time, int material, int object);
    void identify_short_material();
    bool is_assigned_pickup;
    std::vector<int> material_onroute;
    std::vector<int> material_shortage;
};
