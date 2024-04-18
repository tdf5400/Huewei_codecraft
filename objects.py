"""
操作对象相关代码
"""

import numpy as np
import queue

class Berth:
    """ 泊位 """
    def __init__(self, id: int, loc: list, speed, loc_scores, go_time=0) -> None:
        self.raw_loc = np.array(loc)  # 系统位置，在4x4区域的左上角 [x, y]
        if len(loc_scores):
            self.loc = np.array(loc_scores[0])   # 提供接口的位置，该位置的得分比较高
        else:
            self.loc = np.array(loc)
        self.id = np.array(id)  # 唯一id 0~9
        self.speed = speed  # 每帧可以装载的物品数
        self.go_time = go_time  # 轮船从泊口到虚拟点之间所需用时
        self.assigned_robots = []  # 指派的机器人id
        self.alive = True  # 后期泊位到虚拟点的时间超过总时间，则意味着该泊口不可用
        
        self._lock = -1  # 一个泊位最多一艘船，lock为船的id
        self.__cargo = queue.Queue() # 货物(私有)
        self.loc_scores = loc_scores    # 按分数从高到低排序的泊口坐标，分数越高周边障碍物越少(由map.py的berth_locs_by_score算得)
        self.sum_values = 0
        self.arrive_frame = 0

    def lock(self, target: int) -> None:
        """
        加锁
        Args:
            target (_type_): 要求锁定的船id
        """
        self._lock = target

    def unlock(self) -> None:
        """ 解锁 """
        self._lock = -1

    def locked(self) -> bool:
        """ 是否被锁定 """
        return self._lock >= 0
    
    def for_boat(self) -> int:
        """
        查询被哪个船锁定
        Returns:
            int: -1 - 未锁定
                 其他 - 船id
        """
        return self._lock
    
    def has_boat(self) -> bool:
        """ 是否有船在 """
        return 
        
    def is_area(self, loc):
        """ 判断坐标是否在区域里 """
        subs = np.array(loc) - self.loc
        if 0 < subs[0] < 4 and 0 < subs[1] < 4:
            return True
        return False
    
    def put_cargo(self, value: int):
        """ 获得货物 """
        self.__cargo.put(value)
        self.sum_values += value
        
    def clean_cargo(self):
        """ 清空货物信息 """
        self.__cargo.queue.clear()
        
    def value(self):
        """ 读取货物价值 """
        return sum(self.__cargo.queue)
    
    def count(self):
        """ 读取货物数量 """
        return self.__cargo.qsize()
    
    def get_cargos(self, last_size:int):
        """ 
        输出一次物品 
        @param last_size: 最后的空间，比如说船剩3个了，但是泊口有4个，这时候就之会给3个
        """
        pack = []
        # 计算可取次数
        times = min(self.speed, last_size, self.count())

        for i in range(times):
            pack.append(self.__cargo.get())
        return pack


class Robot:
    """ 机器人 """

    def __init__(self, id: int, loc: list, goods=0, status=1) -> None:
        self.loc = np.array(loc)    # 位置 [x, y]
        self.id = id                # 唯一id 0~9
        self.goods = goods          # 是否携带物品 [0-未携带，1-携带]
        self.status = status        # 状态 [0-恢复状态，1-正常运行]
        self._path = []   # 路径，为堆结构(先进后出)，最先执行的在队尾，方便避障
        self._avoid_path = []    # 避障路径，优先执行该路径
        self._taken_cargo = None    # 当前携带的物品
        self.target = []  # 目标
        self.alive = True  # 用于判断机器人是否生成在一个完全封闭区域
        self.assigned_berth = -1  # 机器人被指派的渡口

    def update(self, goods, loc: list, sta) -> None:
        """
        更新数据
        Args:
            goods (_type_): 是否携带物品 [0-未携带，1-携带]
            loc (list): 位置 [x, y]
            sta (_type_): 状态 [0-恢复状态，1-正常运行]
        """
        self.goods = goods
        self.loc = np.array(loc)
        self.status = sta
        
    def value(self):
        """ 当前货物价值 """
        if self._taken_cargo is None:
            return 0
        return self._taken_cargo.value

    def _move(self, direct, is_str: bool = True) -> str:
        """
        机器人移动
        Args:
            direct (_type_): 运动方向：w,a,s,d - 上，左，下，右，或控制的实际int数值
        Returns:
            str: 控制命令
        """
        cmd_direct = {  # 方向对应编号
            'w': 2,  # 上
            'a': 1,  # 左
            's': 3,  # 下
            'd': 0,  # 右
        }
        if is_str:  # 为wsad
            cmd = "move %d %d" % (self.id, cmd_direct[direct])
        else:
            cmd = "move %d %d" % (self.id, direct)
        return cmd

    def get(self) -> str:
        """ 拿取 """
        cmd = "get %d" % (self.id)
        return cmd

    def pull(self) -> str:
        """ 放置 """
        cmd = "pull %d" % (self.id)
        return cmd

    def do(self, cmd: str):
        """ 
        控制行动
        wasd - 移动
        g - 拿取
        p - 放置
        """
        cmd = cmd[0]
        if cmd == 'g':
            return self.get()
        elif cmd == 'p':
            return self.pull()
        else:
            return self._move(cmd, is_str=True)
        
    def set_path(self, path:list):
        """ 设置机器人路径 """
        self._path = path
        
    def get_path(self):
        """ 
        获取机器人路径 
        ps: 为堆结构(先进后出)，最先执行的在队尾，方便避障
        """
        return self._path
    
    def next_path(self):
        """ 
        当前目标点 
        @return None: 没路了
                其他: 目标点
        """
        if self.no_path():
            return None
        return self._path[-1]
    
    def pop_path(self):
        """ 
        获取当前目标点，并弹出 
        @return None: 没路了
                其他: 目标点
        """
        if self.no_path():
            return None
        return self._path.pop()  
    
    def append_path(self, path):
        """ 添加路径（在现有路径的末端加入） """
        # 压入多个值
        if isinstance(path, list): 
            self._path = self._path + path
        # 压入一个值
        else:
            self._path.append(path)        
        
    def size_path(self):
        """ 路径的长度 """
        return len(self._path)
    
    def no_path(self):
        """ 是否没路了 """
        return len(self._path) < 1
    
    def set_avoid(self, path:list):
        """ 设置机器人避障路径 """
        self._avoid_path = path
    
    def append_avoid(self, path):
        """ 添加避障路径 """
        # 压入多个值
        if isinstance(path, list): 
            self._avoid_path = self._path + path
        # 压入一个值
        else:
            self._avoid_path.append(path)  
            
    def get_avoid(self):
        """ 获得避障路径 """
        return self._avoid_path
    
    def pop_avoid(self):
        """ 
        获取当前避障路径目标点，并弹出 
        @return None: 没路了
                其他: 目标点
        """
        if len(self._avoid_path) < 1:
            return None
        return self._avoid_path.pop()
    
    def next_avoid(self):
        """ 
        获得最新的避障目标点 
        @return None: 没路了
                其他: 目标点
        """
        if len(self._avoid_path) < 1:
            return None
        return self._avoid_path[-1]
    
    def size_avoid(self):
        """ 避障路径的长度 """
        return len(self._avoid_path)
    
    def no_avoid(self):
        """ 是否避障没路了 """
        return len(self._avoid_path) < 1
        
    

        
        

class Boat:
    """ 船 """

    def __init__(self, id: int, loc: list, capacity=1000, status=0, target=-1) -> None:
        self.loc = np.array(loc)  # 位置 [x, y]
        self.id = id  # 唯一id 0~4
        self.capacity = capacity  # 容积，可装载数量 1~1000
        self._used = 0  # 已用容积
        self._value = 0 # 携带物品价值
        self.status = status  # 状态，[0-移动(运输中)，1-正常运行状态(即装货状态或运输完成状态)，2-泊位外等待状态]
        self.target = target  # 目标泊位，若目标泊位为虚拟点，则为-1
        self._changed = False  # 本帧是否做了移动，若移动则不会搬货，每帧update的时候会被重置

    def update(self, sta, target) -> None:
        """
        更新数据
        Args:
            sta (_type_): 状态，[0-移动(运输中)，1-正常运行状态(即装货状态或运输完成状态)，2-泊位外等待状态]
            target (_type_): 目标泊位，若目标泊位为虚拟点，则为-1
        """
        self.status = sta
        self.target = target
        self._changed = False   # 重置状态

    def move(self, target: int) -> str:
        """
        船移动
        Args:
            target (int): [0~9 - 目标泊位， -1 - 驶离泊位售卖物品]
        """
        if target >= 0:
            cmd = "ship %d %d" % (self.id, target)
        else:
            cmd = "go %d" % (self.id)
        self._changed = True
        return cmd
    
    def get(self, vals):
        """ 获得货物 """
        if isinstance(vals, int):    # 单个值
            vals = [vals]
        for val in vals:
            # 装满退出
            if self._used == self.capacity:
                raise IOError("Boat cannot get too much cargo")
            
            self._value += val
            self._used += 1
            
    def size(self) -> int:
        """ 当前货物量 """
        return self._used
    
    def free(self) -> int:
        """ 剩余载货量 """
        return self.capacity - self._used
            
    def full(self) -> bool:
        """ 是否满了 """
        return self._used == self.capacity
    
    def clear(self):
        """ 清空货物 """
        self._value = 0
        self._used = 0

    def value(self):
        """ 当前价值 """
        return self._value
        
    def is_arrive(self):
        """ 船是否到达 """
        return self.status != 0
    
    def is_changed(self):
        """ 本帧是否移动 """
        return self._changed
        

class Cargo:
    """ 货物 """

    def __init__(self, loc: list, value: int, born_frame: int, berths_dist:list=[]) -> None:
        self.loc = np.array(loc)  # 位置
        self.value = value  # 价值 1~200
        self.born_frame = born_frame  # 出生时间(帧)
        self._lock = -1  # 拿取锁[-1 - 未锁定, 其他-被预定的机器人编号]
        self.berths_dist = berths_dist # 长度为berth_num的数组，意思到各个泊口的距离，若无法到达则为-1
        self._taken = False # 是否已经被拿走，True - 被拿走了

    def lock(self, target: int) -> None:
        """
        加锁
        Args:
            target (_type_): 要求锁定的机器人id
        """
        self._lock = target

    def unlock(self) -> None:
        """ 解锁 """
        self._lock = -1

    def locked(self) -> bool:
        """ 是否被锁定 """
        return self._lock >= 0
    
    def for_bot(self) -> int:
        """
        查询被哪个机器人锁定
        Returns:
            int: -1 - 未锁定
                 其他 - 机器人id
        """
        return self._lock
    
    def take(self) -> None:
        """ 设置为已拿取 """
        self._taken = True
    
    def is_taken(self) -> bool:
        """ 是否已经被拿走 """
        return self._taken
    
    def to_berth(self, berth_id) -> bool:
        """ 是否可以到达某个泊口 """
        return self.berths_dist[berth_id] >= 0