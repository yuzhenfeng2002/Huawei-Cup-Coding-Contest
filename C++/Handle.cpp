#include "Handle.h"

Handle::Handle(int id)
{
    this->id = id;
    this->handle_type = 0;
    this->x = 0;
    this->y = 0;
    this->left_time = -1;
    this->material = 0;
    this->object = 0;
    this->material_onroute = std::vector<int>(HANDLE_TYPE_NUM, 0);
}

void Handle::update(int handle_type, int x, int y, int left_time, int material, int object)
{
    this->handle_type = handle_type;
    this->x = x;
    this->y = y;
    this->left_time = left_time;
    this->material = material;
    this->object = object;
    identify_short_material();
}
// 这里必须用二进制判断吗？
void Handle::identify_short_material()
{
    std::bitset<HANDLE_OBJECT_NUM> material_info(this->material);
    this->material_shortage.clear();
    for (int i = 0; i < 3; ++i)
    {
        int m = TYPE_MATERIAL[this->handle_type][i];
        if (material_info[HANDLE_OBJECT_NUM - m] + this->material_onroute[m - 1] == 0)
        {
            this->material_shortage.push_back(m);
        }
    }
}