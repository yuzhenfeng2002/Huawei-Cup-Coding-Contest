# 这是一个名为Handle的类，它代表了一个可操作的物体，包含一些属性和方法：

# 属性：

# id: 物体的唯一标识符
# handle_type: 物体的类型
# x, y: 物体的坐标
# left_time: 物体离开的剩余时间
# material: 物体上的物料
# object: 物体上的对象
# is_assigned_pickup: 物体是否已被指派
# material_onroute: 物体正在路上的物料
# 方法：

# update: 更新物体的属性
# identify_short_material: 识别缺少的物料
# 在update方法中，给定物体的类型、坐标、离开时间、物料和对象，更新物体的属性。


# 在identify_short_material方法中，将物体上缺少的物料识别出来。
from Params import *
import numpy as np
class Handle:

    def __init__(self, id) -> None:
        self.id = id
        self.handle_type = 0
        self.x = 0
        self.y = 0
        self.left_time = -1
        self.material = 0
        self.object = 0

        self.is_assigned_pickup = 0
        self.material_onroute = np.zeros(HANDLE_TYPE_NUM)

    def update(self, handle_type, x, y, left_time, material, object):
        self.handle_type = handle_type
        self.x = x
        self.y = y
        self.left_time = left_time
        self.material = material
        self.object = object
        self.identify_short_material()

    def identify_short_material(self):
        material_info = "{:08b}".format(self.material)
        self.material_shortage = []
        for m in TYPE_MATERIAL[self.handle_type]:
            if int(material_info[HANDLE_OBJECT_NUM -
                                 m]) + self.material_onroute[m - 1] == 0:
                self.material_shortage.append(m)
