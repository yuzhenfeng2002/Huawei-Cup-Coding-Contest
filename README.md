# Huawei Cup Coding Contest

## 运行测试

python版本的命令为：

```shell
./Robot -m maps/1.txt -c ./SDK/python "python3 main.py"
```

```shell
./Robot_gui -m maps/1.txt -c ./my "python main.py"
```

## 文件结构

main.py
主要执行函数

Params.py
机器人参数和地图参数声明

Map.py
策略选择和机器人运动轨迹规划

## 主要类

### Handle类

属性：

id: 物体的唯一标识符

handle_type: 物体的类型

x, y: 物体的坐标

left_time: 物体离开的剩余时间

material: 物体上的物料

object: 物体上的对象

is_assigned_pickup: 物体是否已被指派

material_onroute: 物体正在路上的物料

方法：

update: 更新物体的属性

identify_short_material: 识别缺少的物料

在update方法中，给定物体的类型、坐标、离开时间、物料和对象，更新物体的属性。

在identify_short_material方法中，将物体上缺少的物料识别出来。

### Robot

id：机器人的标识符。

delta_time：更新之间的时间差，根据一个名为FPS的常量计算得出。

handle：对机器人的引用。

object_type：机器人物品类型的标识符。

time_coeff：时间系数。

crash_coeff：碰撞系数。

rotate_speed：机器人旋转速度。

speed_x：机器人沿x轴移动的速度。

speed_y：机器人沿y轴移动的速度。

direction：机器人面朝的方向。

x：机器人当前位置的x坐标。

y：机器人当前位置的y坐标。

is_assigned_task：标志机器人是否有任务分配。

x_：任务的x坐标。

y_：任务的y坐标。

task_list：机器人要完成的任务列表。

strategy_dict：机器人的策略字典。

last_assigned_time：机器人最后一次接收到任务的时间。

该类还包含以下方法：

__init__：初始化机器人的属性。

update：更新机器人的属性。

add_task：向机器人的任务列表中添加一个新的任务。

strategy：确定机器人的策略。

arrive：机器人到达任务位置时执行的操作。

### Map

__init__(self): 类的初始化函数，设置了一些基本的地图属性，如当前帧数、初始金钱、机器人列表、抓取器列表、抓取器类型字典等。
update_map(self, frame, money): 更新地图的帧数和金钱。

init_robots(self): 初始化机器人列表。

init_handles(self, num): 初始化抓取器列表。

update_robot(self, id, handle_id, object_type, time_coeff, crash_coeff, rotate_speed, speed_x, speed_y, direction, x, y): 更新机器人的状态。

update_handle(self, id, handle_type, x, y, left_time, material, object): 更新抓取器的状态。

update_handle_type_dict_first(self): 根据抓取器类型将抓取器列表分类。

strategy_to_str(self, robot: Robot): 将机器人的策略转化为字符串格式。

output_strategy(self): 输出机器人的策略。

get_short_material(self): 获取缺货的原材料列表。

set_robots_targets(self): 设置机器人的任务和目标位置。

其中，类变量包括：

frame: 当前帧数。

money: 当前金钱。

robot_list: 机器人列表。

handle_list: 抓取器列表。

handle_type_dict: 抓取器类型字典

### 复赛地图读取和路径规划

#### 地图读取

地图中”#“代表障碍物，每个节点大小为**0.5m**。先通过代码将地图转换为01数组，0代表空地，1代表障碍物节点。

```python
import matplotlib.pyplot as plt
map_array = []
with open(file_path, 'r') as f:
    for line in f:
        row = [1 if c == '#' else 0 for c in line.strip()]
        map_array.append(row)
# 绘制栅格图
plt.imshow(map_array, cmap='binary', interpolation='nearest')

# 显示栅格图
plt.show()
# print(map_array)
```

#### 路径规划

目前写了一个A*算法路径规划，从结果来看还算有效，之后再提高效率。示例文件放在了`tran.py`中。
