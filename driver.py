import numpy as np
import sys
import queue
from tree import QuadTree   # 节点树
from map import *       # 地图
from objects import *   # 物品对象
import multiprocessing
import time

import debug

"""
TODO
ok. 货物过期时清出队列
2. 地图索引优化
ok. A*寻路
4. 避障可以根据对方要去的地点行动
5. 路径重算放多线程里
"""

""" 参数设置 """
BOT_TIMEOUT = 20   # 机器人卡死帧数判定阈值
NUM_PROCESS = 2    # 同时运行的最大进程数


""" 基础对象定义 """
berth_num = 10
bot_num = 10
boat_num = 5
cargo_alive = 1000  # 货物存活帧

ACTION_BOT_WALK = 0 # 机器人行走
ACTION_BOT_GET =  1 # 机器人拿取
ACTION_BOT_PULL = 2 # 机器人释放

_task_pool = None   # 多进程的pool

def _process_fail(e):
    """ 多进程任务报错信息 """
    raise IOError(e)

def find_loc_in_list(objs:list, loc):
    """ 
    寻找坐标对应的对象 
    :return -1 - 查找失败
           其他 - 对应下标
    """
    loc = np.array(loc)
    for i, obj in enumerate(objs):
        if (obj.loc == loc).all():
            return i
    return -1

def find_nearst_in_list(objs:list, loc):
    """ 
    寻找与坐标最近的对象 
    @return : [距离, idx]
    """
    loc = np.array(loc)
    dists = []
    
    for i, obj in enumerate(objs):
        dist = np.sum(np.abs(obj.loc - loc))
        dists.append([dist, i])

    dists.sort(key=lambda x:x[0])
    return dists[0]

def _process_bot_search_path(bot_id, bots, map, end_loc, dfs_map=None, find_bots=False):
    """ 
    机器人寻路进程 
    @param find_bots: 是否要寻找隔壁的机器人
    @param is_berths: 起始坐标和
    """
    bot = bots[bot_id]
    map = Map(map)  # 格式化一下，否则调用类方法会出错
    # 寻找隔壁的机器人
    neighbors = []
    if find_bots:
        neighbors_hash = set()
        for b in bots:
            # # 不考虑比自己低的
            # if b.id < bot_id:
            #     continue
            # 记录其坐标及其2格里的目标点
            if np.sum(np.abs(bot.loc - b.loc)) < 2:
                neighbors_hash.add(loc2hash(b.loc))
                
                nb_path = bots[b.id].get_path()
                for i in nb_path[-1*min(len(nb_path), 2):]:
                    neighbors_hash.add(loc2hash(i))
        # 删去当前机器人位置
        neighbors_hash.discard(loc2hash(bot.loc))
        # hash转loc
        neighbors = [hash2loc(i) for i in neighbors_hash]
    
    # 寻路
    start_loc = np.array(end_loc)
    target_loc = bots[bot_id].loc
        
    
    # 可以用dfs
    if dfs_map is not None: 
        start_dist = dfs_map[start_loc[1]][start_loc[0]]
        target_dist = dfs_map[target_loc[1]][target_loc[0]]    

        if start_dist >= 0 and target_dist >= 0:    
            # start为泊口（这里不用0判定是因为泊口为4x4范围）
            if start_dist < target_dist:
                # 寻路
                dfs_path = path_search_dfs(dfs_map, target_loc, obstacles=neighbors)
                if isinstance(dfs_path, list):
                    dfs_path = dfs_path[::-1] # 反转路径  
            # target为泊口
            else:
                dfs_path = path_search_dfs(dfs_map, start_loc, obstacles=neighbors)  
                
            # 寻路成功
            if isinstance(dfs_path, list):  
                # 如果目标点距离泊口位置还有距离，则删去泊口的部分用A*补上
                if min(start_dist, target_dist) != 0:
                    # 最后一点路用a*算
                    idx = min(3, len(dfs_path)-1) # 删掉最后多少格
                    if start_dist < target_dist:    # start为泊口
                        # A*寻路
                        as_path = path_search_a_star(map, 
                            start=start_loc, target=dfs_path[idx], 
                            max_step=50, obstacles=neighbors)
                        # A*寻路成功（一般不会失败吧）
                        if isinstance(as_path, list):
                            path = as_path + dfs_path[idx+1:]
                            return [bot_id, path]
                    else:                           # target为泊口
                        # A*寻路
                        as_path = path_search_a_star(map, 
                            start=dfs_path[-idx], target=target_loc, 
                            max_step=50, obstacles=neighbors)
                        # A*寻路成功（一般不会失败吧）
                        if isinstance(as_path, list):
                            path = dfs_path[:-idx] + as_path
                            return [bot_id, path]
                else:
                    path = dfs_path
                    return [bot_id, path]
                        
    # 只能用A* / dfs失败   
    path = path_search_a_star(map, start=start_loc, target=target_loc, max_step=300, obstacles=neighbors)
    return [bot_id, path]

""" 驱动部分 """
class Driver:
    """ 交互驱动 """
    frame = 0  # 帧号(每帧更新)
    money = 0  # 金钱数(每帧更新)
    map = Map()  # 地图
    map_codes = np.array([800853.41, 311964.104])  # 每个地图的code
    map_id = -1     # 地图id
                    # -1 - 未知
                    # 1  - map1
                    # 2  - map2
    berths = [Berth(i, [0,0], 0, []) for i in range(berth_num)]  # 泊位(10个)
    berths_dfs_map = [None for _ in range(berth_num)]       # 泊位的dfs距离
    bots = [Robot(i, [0,0]) for i in range(bot_num)]        # 机器人(10个)
    boats = [Boat(i, [0,0]) for i in range(boat_num)]       # 船(5个)
    cargos = queue.Queue()      # 货物
    commands = queue.Queue()    # 指令缓存
    task_list = []  # 任务列表，往里边加要执行的函数 [id, task]
    task_init_list = []  # 初始化时要执行的任务列表 [id, task]
    task_next_frame = [] # 下一帧要执行的任务 [id, task] # 执行完会自动清除
    _bot_last_update = [0 for _ in range(berth_num)] # 机器人上次到达目标点的帧数，超时用
    _bot_researching = set()    # 当前在寻路的机器人id =[10+id]
    _bot_timeout = [0 for _ in range(berth_num)]    # 机器人超时次数
    _bot_id2sup = [x for x in range(bot_num)]    # 机器人优先级列表，i位置显示机器人i的优先级排序位置
    _bots_by_sup = bots # 按照优先级指向self.bots

    def __init__(self, init: bool = False) -> None:
        """
        对象构造
        Args:
            init (bool, optional): 是否在构造时进行初始化. Defaults to False.
        """
        if init:
            self.initialize()

    def initialize(self) -> None:
        """ 初始化 """
        t_0 = time.perf_counter()   # 计时
        t_timeout = t_0 + 4.5 # 最大结束时间
        
        global _task_pool
        _task_pool = multiprocessing.Pool(NUM_PROCESS)  # 进程池
        
        self.map_str = []  # 地图字符串
        # 记录地图信息
        for i in range(0, map_size[0]):
            line = input()
            self.map_str.append(line)  
        # 初始化地图
        self.map, bot_locs = str2map(self.map_str)

        # 解析泊位信息
        for i in range(berth_num):
            line = input()
            berth_list = [int(c) for c in line.split(sep=" ")]
            id = berth_list[0]
            self.berths[id] = Berth(
                id, loc=[berth_list[2], berth_list[1]], 
                speed=berth_list[4], 
                loc_scores=berth_locs_by_score([berth_list[2], berth_list[1]], self.map),
                go_time=berth_list[3])
        # 解析轮船容积
        capacity = int(input())
        for i in range(boat_num):
            self.boats[i] = Boat(i, [0, 0], capacity)

        """ 信息预处理 """
        # 初始化机器人位置
        for id in range(bot_num):
            self.bots[id] = Robot(id, bot_locs[id])
            
        # 计算机器人优先级
        self._bot_id2sup = [x for x in range(bot_num)]    # 机器人优先级列表，i位置显示机器人i的优先级排序位置
        self._bots_by_sup = self.bots # 按照优先级指向self.bots
        
        # 计算各个泊口的dfs梯度
        _task_pool.apply_async(dfs_genMap, args=(self.map, [self.berths[i].loc for i in range(6, berth_num)]), callback=self._callback_dfs_init)  # 一半放另一个进程里算
        self.berths_dfs_map[:6] = dfs_genMap(self.map, [self.berths[i].loc for i in range(6)])   
        
        # 解析地图信息
        map_code = (np.var([b.loc[0] for b in self.berths]) + np.var([b.loc[1] for b in self.berths])) * np.mean([b.loc[0] for b in self.berths])
        for id, c in enumerate(self.map_codes):
            if abs(c - map_code) < 50:
                self.map_id = id + 1
                break
        debug.log(f'map_code:{map_code}, id:{self.map_id}')
            
        # 执行初始化任务
        for id, task in self.task_init_list:
            task(self)
            
        # 指派机器人dfs到分配泊口，以防止开帧大量计算
        for bot in self.bots:
            if bot.alive:
                self.bot_goto(bot.id, self.berths[bot.assigned_berth].loc)

        while time.perf_counter() < t_timeout and self.berths_dfs_map[-1] is None:
            pass

        # 计时
        debug.log("Init time: %.2f ms" % ((time.perf_counter()-t_0)*1000))
        debug.log('dfs result: ', [i is not None for i in self.berths_dfs_map])

        """ 初始化结束 """
        okk = input()  # 最后一行ok
        print("OK")
        sys.stdout.flush()

    def frame_update(self) -> None:
        """ 帧更新 """
        self.frame, self.money = map(int, input().split(" "))
        # 更新货物信息
        num = int(input())  # 新增货物数量
        t0 = time.perf_counter() # 本帧开始时间
        t_timeout = t0 + 0.013 # 过期时间
        for i in range(num):
            y, x, val = map(int, input().split())
            berths_dist = []
            # 查询能否到达泊口，若都不能，则不添加
            for berth_map in self.berths_dfs_map:
                if berth_map is None or berth_map[y][x] < 0: # dfs还没算完/不可到达
                    berths_dist.append(-1)
                else:
                    berths_dist.append(berth_map[y][x])
                    
            if berths_dist.count(-1) < len(berths_dist):    # 不全为-1
                self.cargos.put(Cargo(loc=[x, y], value=val, born_frame=self.frame, berths_dist=berths_dist))
        # 更新机器人信息
        for i in range(bot_num):
            goods, y, x, sta = map(int, input().split())
            self.bots[i].update(goods, [x, y], sta)
            
        # 更新船信息
        for i in range(boat_num):
            # 船状态更新
            sta, target = map(int, input().split())
            self.boats[i].update(sta, target)

            # 船到达售卖点后清空信息
            if self.boats[i].is_arrive() and self.boats[i].target == -1:
                self.boats[i].clear()
        # 帧尾的OK
        okk = input()
        
        debug.log(f'frame: {self.frame}')
        
        # 执行上一帧所留下的任务
        for id, task, args in self.task_next_frame:
            task(*args)
        self.task_next_frame = [] # 清空任务

        # 清出过期货物
        alive = self.frame - cargo_alive  # 过期帧 = 当前帧 - 存活时间
        while (not self.cargos.empty()) and (self.cargos.queue[0].born_frame < alive):
            self.cargos.get()
        # debug.log(f'to cargo: {(time.perf_counter()-t0)*1000}' )
              
        # 执行每帧任务
        for id, task in self.task_list:
            task(self)            
        # debug.log(f'to task: {(time.perf_counter()-t0)*1000}' )
            
        # 避障
        self._bot_avoid()
        # debug.log(f'to _bot_avoid: {(time.perf_counter()-t0)*1000}' )
            
        # 进行机器人运维
        self._ctl_bots()
        # debug.log(f'to _ctl_bots: {(time.perf_counter()-t0)*1000}' )
        
        # 泊口搬运货物      
        for i, boat in enumerate(self.boats):
            if boat.is_changed(): # 本帧移动，拿不到货物
                continue
            flag = self._boat_get_cargo(i)
        # debug.log(f'to _boat_get_cargo: {(time.perf_counter()-t0)*1000}' )
            
        # 执行指令列表
        while not self.commands.empty():
            cmd = self.commands.get()
            print(cmd, end='\n')
        sys.stdout.flush()
        # debug.log(f'to cmd: {(time.perf_counter()-t0)*1000}' )
            
        # 有任务时，等到本帧结束再释放
        while len(self._bot_researching) and time.perf_counter() > t_timeout:  # 空运算等待
            pass
        # debug.log('self._bot_researching', self._bot_researching)
            
        # used_time = time.perf_counter()-t0
        # debug.log('used_time: %.4f' % (used_time*1000))
        
        b0 = self.bots[0]

        # 结束本帧
        print("OK")
        sys.stdout.flush()

    def add_task(self, task, init: bool = False) -> None:
        """
        添加每帧都要做的事情，会传入本对象句柄进去

        Args:
            task (_type_): 任务函数，应留传入接口，如func(self)
            init (bool, optional): 仅在初始化时执行. Defaults to False.
        """
        if init:
            task_len = len(self.task_init_list)
            self.task_init_list.append([task_len, task])
        else:
            task_len = len(self.task_list)
            self.task_list.append([task_len, task])
             
    def bot_goto(self, bot_id, target_loc, thread=False, find_neightbors=False):
        """ 机器人移动到指定目标 
        @param bot_id: 机器人的id
        @param target_loc: 目标点
        @param thread: 是否要多线程跑
        @param find_neightbors: 是否要考虑到周围的机器人
        
        Returns:
        int:  
             0 - 成功
            -1 - 起始/目标点不为路
            -2 - 寻路失败
            -3 - 机器人已经在目标点相同
        """
        if (self.bots[bot_id].loc == target_loc).all():
            return -3
        
        # 1. 启动计算
        task_name = 10+bot_id
        if not task_name in self._bot_researching:
            # 寻找能不能用dfs
            dfs = None
            start_loc = self.bots[bot_id].loc
            if self.map.get_sign(target_loc) == 'B':
                berth_idx = find_nearst_in_list(self.berths, target_loc)[1]
                dfs = self.berths_dfs_map[berth_idx]
            elif self.map.get_sign(start_loc) == 'B':
                berth_idx = find_nearst_in_list(self.berths, start_loc)[1]
                dfs = self.berths_dfs_map[berth_idx]
                
            # 多线程
            if thread:
                global _task_pool   
                # 标记本机器人
                self._bot_researching.add(task_name) 
                # 开始进程
                _task_pool.apply_async(
                    _process_bot_search_path, 
                    args=(bot_id, self.bots, self.map, target_loc, dfs, find_neightbors), 
                    callback=self._callback_bot_research, 
                    error_callback=_process_fail
                )   
                
            # 单线程
            else:
                # 标记本机器人
                self._bot_researching.add(task_name) 
                # 开始进程
                self._callback_bot_research(_process_bot_search_path(bot_id, self.bots, self.map, target_loc, dfs, find_neightbors))
                
        return 0         
  
    def boat_goto(self, boat, target):
        """ 
        船移动 
        @param 可为船的对象/id
        @param target: 泊口id，若为-1则驶离
        
        @ return: 0 - 成功
                 -1 - 目标泊口被占用
                 -2 - 已经为该目标
        """
        if isinstance(boat, int):
            boat = self.boats[boat]
        
        # 已经为该目标
        if boat.target == target:
            return -2

        # 离开
        if target == -1:
            # 解锁泊口
            self.berths[boat.target].unlock()

        # 去泊口
        else:
            # 目标泊口被占用
            if self.berths[target].locked():
                return -1
        
            if boat.target != -1: # 从别的泊口来
                # 解锁泊口
                self.berths[boat.target].unlock()
            # 锁定泊口
            self.berths[target].lock(boat.id)
            
        # 发送命令(这里不能用boat，会导致不刷新)
        self._cmd(self.boats[boat.id].move(target))
             
    def get_berth_distance(self, berth_id, loc):
        """
        获取到达某个泊口的距离
        @return >=0 距离
                <0  无法到达
        """
        if self.berths_dfs_map[berth_id] is None:   # 还没初始化好
            return -1
        return self.berths_dfs_map[berth_id][loc[1]][loc[0]]
        
             
    """ 内部运维部分 """               
    def _cmd(self, command: str) -> None:
        """ 加入要执行的指令 """
        self.commands.put(command)
        
    def _add_next_frame_task(self, task, args) -> None:
        """ 
        添加下一帧要做的事情，执行完会清空；
        供对象成员使用，一般做简单验证而已，最好不要有大量计算 
        """
        task_len = len(self.task_next_frame)
        self.task_next_frame.append([task_len, task, args])
    
    def _path_search(self, start_loc, target_loc, obstacles=[]):
        """ 寻路 """        
        # 1.0. 寻路 - A*
        # path = path_search_a_star(self.map, start=start_loc, target=target_loc, max_step=500, obstacles=obstacles)
        
        # 1.0. 寻路 - dfs + A*
        start_2_berth = find_loc_in_list(self.berths, start_loc)
        target_2_berth = find_loc_in_list(self.berths, target_loc)
        # 起始点和目标点有一个在泊口，使用dfs
        if start_2_berth != -1 or target_2_berth != -1:
            berth_id = max(start_2_berth, target_2_berth)   # 过去/离开的泊口id
            # 起点为泊口
            if start_2_berth != -1:
                path = path_search_dfs(self.berths_dfs_map[berth_id], target_loc, obstacles=obstacles)
                if not isinstance(path, int):
                    path = path[::-1] # 反转路径
            # 终点为泊口
            else:
                path = path_search_dfs(self.berths_dfs_map[berth_id], start_loc, obstacles=obstacles)  
                
            # 寻路成功则退出
            if not isinstance(path, int):
                return path
                
        # 起始点和目标点都不在泊口 / 寻路失败，使用A*
        path = path_search_a_star(self.map, start=start_loc, target=target_loc, max_step=300, obstacles=obstacles)
            
        return path
        
    def _bot_verify(self, bot_id, bot_action, target):
        """ 机器人事件验证 """        
        # 1. 机器人拿取判断
        if bot_action == ACTION_BOT_GET:    
            if self.bots[bot_id].goods == 1: # 携带物品，说明成功
                self.bots[bot_id].set_path([]) # 清空指令
                cargo_id = find_loc_in_list(self.cargos.queue, target)  # 寻找货物id
                # 拿取货物
                self.cargos.queue[cargo_id].take()  # 把对应物品标记拿取
                self.bots[bot_id]._taken_cargo = self.cargos.queue[cargo_id]  # 机器人携带
            else: # 拿取失败，检查附近有没有货物，没有则可能消失了，弹出
                result = find_loc_in_list(self.cargos.queue, self.bots[bot_id].loc)
                if result == -1:    # 确实没有
                    self.bots[bot_id].set_path([]) # 清空指令
                    
        # 2. 机器人释放判断
        elif bot_action == ACTION_BOT_PULL:
            if self.bots[bot_id].goods == 0: # 不携带物品，说明成功
                self.bots[bot_id].set_path([]) # 清空指令
                berth_id = find_nearst_in_list(self.berths, target)[1]
                # 放置货物
                cargo = self.bots[bot_id]._taken_cargo 
                if self.bots[bot_id]._taken_cargo is None:
                    raise IOError(f'{self.bots[bot_id]._taken_cargo}, {self.bots[bot_id].goods}')
                self.berths[berth_id].put_cargo(cargo.value)    # 泊口信息更新
                self.bots[bot_id]._taken_cargo = None   # 机器人取消携带
            else: # 拿取失败，检查附近有没有货物，没有则可能消失了，弹出
                result = find_loc_in_list(self.cargos.queue, self.bots[bot_id].loc)
                if result == -1:    # 确实没有
                    self.bots[bot_id].set_path([]) # 清空指令


    def _has_rob(self,pose) -> None:
        """
       检测位置是否有机器人,无返回0 ,有返回机器人id+1
        """    
        for b in self.bots:
            if b.no_path():
                continue
            if b._path[-2][0] == pose[0] and b._path[-2][1] == pose[1]:
                return b.id+1
            # if b.loc[0] == pose[0] and b.loc[1] == pose[1]:
            #     return b.id+1

        return 0


    def _bot_avoid(self):
        """
        避障模块,通过修改bot的path以实现避障
        """  


        
        # # 1.卡死判定
        #     # 无路径，跳过
        #     if bot.no_path():
        #         continue  
            
        #     task_name = 10 + bot_id
        #     if task_name in self._bot_researching:   # 已经在寻路，跳过
        #         continue
            
        #     # 卡死判定，重新寻路 (BOT_TIMEOUT乘id是为了保证timeout时间不一样)
        #     if self.frame - self._bot_last_update[bot_id] > BOT_TIMEOUT*(1+0.1*bot_id):
        #         debug.log(f'bot {bot_id} stuck: {self.bots[bot_id].loc} to {self.bots[bot_id].get_path()[0]}')
        #         # 寻路三次失败，则回自己泊口
        #         if self._bot_timeout[bot_id] > 3:
        #             target_loc = self.berths[bot.assigned_berth].loc if bot.assigned_berth>=0 else bot.get_path()[0]
        #         # 正常寻找解法
        #         else:
        #             target_loc = bot.get_path()[0]
                    
        #         # debug.log('bot', bot_id,'research: ', target_loc)
        #         # self.bots[bot_id].set_path([target_loc])  # 清空目标
        #         self.bot_goto(bot_id, target_loc=target_loc, thread=True, find_neightbors=True)
        #         continue


        clear_buf = []
        clear_buf_ = []
        
        # 防止路径过少时导致ctl里面会让机器人额外行动的问题
        for i in range(0,10):
            b = self.bots[9-i]
            if b.no_path():
                path__ = b.get_path()   
                path__.append(b.loc)
                path__.append(b.loc)         
                b.set_path(path__)             
                clear_buf.append(9-i)       
                # continue 
            elif (b.loc == b._path[-1]).all()==False:
                path__ = b.get_path()   
                path__.append(b.loc)
                b.set_path(path__) 
            elif len(b._path)==1 and (b.loc == b._path[-1]).all():
                path__ = b.get_path()   
                path__.append(b.loc)   
                b.set_path(path__)                        
                clear_buf_.append(9-i)         


        # 2.避障
        # 2.1 动态优先级(不用直接屏蔽就好)：计算每个机器人周围的可行区域算优先级
        # if self.frame % 100 ==0:
        if 1:
            available_areas = [cal_nearby_area(self.map, b.loc, 1, self.bots) for b in self.bots]
            areas_sorted = sorted(enumerate(available_areas), key=lambda x:x[1], reverse=True)  # 根据区域数量递增排序
            bot_sup_idx = [x[0] for x in areas_sorted]  # 获取排序好后b坐标,下标在第0位
            self._bot_id2sup = [bot_sup_idx.index(i) for i in range(bot_num)] # 优先级列表，i位置显示机器人i的优先级排序位置
            self._bots_by_sup = [self.bots[i] for i in bot_sup_idx]   # 指向self.bots   

        # 获得优先级信息
        bot_id2sup = self._bot_id2sup
        bot_sup = self._bots_by_sup
        
        


        # bot_sup_idx_ = []
        # for i in range(0,10):
        #     bot = self.bots[bot_sup_idx[i]]
        #     if not bot_sup_idx[i] in bot_sup_idx_:
        #         bot_sup_idx_.append(bot_sup_idx[i])
        #     else:
        #         continue
        #     for j in range(i,10):
        #         bot_ = self.bots[bot_sup_idx[j]]
        #         if np.sum(np.abs(bot.loc - bot_.loc))==1:
        #             if not bot_sup_idx[j] in bot_sup_idx_:
        #                 bot_sup_idx_.append(bot_sup_idx[j])
                        
                        # continue
            


        bot_sup_idx_ = []
        for i in range(0,10):
            bot = self.bots[bot_sup_idx[i]]
            if not bot_sup_idx[i] in bot_sup_idx_:
                bot_sup_idx_.append(bot_sup_idx[i])
            else:
                continue
            for j in range(i,10):
                # if  i>= j:
                #     continue
                bot_ = self.bots[bot_sup_idx[j]]
                if np.sum(np.abs(bot.loc - bot_.loc))==1:
                    if not bot_sup_idx[j] in bot_sup_idx_:
                        dx = bot._path[-2][0] - bot._path[-1][0]  
                        dy = bot._path[-2][1] - bot._path[-1][1]                          
                        dx_  = bot_._path[-2][0] - bot_._path[-1][0]  
                        dy_ = bot_._path[-2][1] - bot_._path[-1][1]       
                        if dx*dx_>=0 and dy*dy_>=0:  
                            bot_sup_idx_.append(bot_sup_idx[j])
                            # continue
                        # else:
                        #     bot_sup_idx_.insert(0,bot_sup_idx[j])                            
                        # if dx*dx_>=0 and dy*dy_>=0:  
                        #     bot_sup_idx_.insert(0,bot_sup_idx[j])
                        # else:
                        #     bot_sup_idx_.append(bot_sup_idx[j])                                          
                            # continue

        # bot_sup_idx_ = bot_sup_idx_[::-1]

        self._bot_id2sup = [bot_sup_idx_.index(i) for i in range(bot_num)] # 优先级列表，i位置显示机器人i的优先级排序位置
        self._bots_by_sup = [self.bots[i] for i in bot_sup_idx_]   # 指向self.bots   

        bot_id2sup = self._bot_id2sup
        bot_sup = self._bots_by_sup
        
        # for bot_id, bot in enumerate(self.bots): 

        #     debug.log('robot'+str(bot_id)+' pose'+str(self.bots[bot_id].loc))
        #     if len(self.bots[bot_id]._path)>0:
        #         for i in range(1, min(8,len(self.bots[bot_id]._path))+1):
        #             debug.log('robot'+str(bot_id)+'  '+str(i)+'robot_path'+str(self.bots[bot_id]._path[-i]))        
        #     else:
        #         debug.log('robot'+str(bot_id)+' nopath')

        self.avoid_buf=[]
        self.avoid_buf.clear()
        for i in range(0,10):
            self.avoid_buf.append(0)
        for i in range(0,10):
            bot = bot_sup[9-i]
            # bot = bot_sup[i]            
            bot_id = bot.id

            if bot.no_path():
                continue  

            path = self.bots[bot_id].get_path()            

            for i in range(0,10):
                # bot = bot_sup[i]
                b = bot_sup[9-i]
                if bot_id2sup[b.id] <= bot_id2sup[bot_id]:      #大的不躲 小的躲
                    continue
                # if bot_id2sup[b.id] >= bot_id2sup[bot_id]:   
                    # continue                
                #self.avoid_buf[bot_id] 为bot id机器人避让躲过最大的机器人的id
                # if bot_id2sup[self.avoid_buf[b.id]] > bot_id2sup[self.avoid_buf[bot_id]]:      #大的不躲 小的躲
                #     continue
                # if self.avoid_buf[bot_id]!=0:
                #     continue
                     
                # debug.log('robot'+str(bot_id2sup[self.avoid_buf[b.id]] )+' nopath'+str(bot_id2sup[self.avoid_buf[bot_id]]))                
                if b.no_path():                   
                    continue           
                # if self.avoid_buf[bot_id]>:
                #     continue

                #俩机器人有相撞的可能
                path_buf = b.get_path()

                # if len(path_buf)<=1:
                #     continue
                # if len(path)<=1:
                #     continue

                avoid = 0 
                dist = np.sum(np.abs(self.bots[bot_id].loc - self.bots[b.id].loc))          
                w =-1

                if 1==1:
                    if dist ==  1:
                        back_ =0
                        dx=0
                        dy=0
                        if (path[-1+w] == path_buf[-1+w]).all():
                            dx = path_buf[-1+w][0] - self.bots[b.id].loc[0]
                            dy = path_buf[-1+w][1] - self.bots[b.id].loc[1]                                    
                            back_ = 1
                        elif (self.bots[bot_id].loc == path_buf[-1+w]).all() and (path[-1+w] == self.bots[b.id].loc).all(): #正面相撞                     
                            dx = path_buf[-1+w][0] - self.bots[b.id].loc[0]
                            dy = path_buf[-1+w][1] - self.bots[b.id].loc[1]          
                            back_=1
                        elif  (path[-1+w] == self.bots[b.id].loc).all():                                      #小号机器人撞大号
                            dx = path[-1+w][0] - self.bots[bot_id].loc[0]
                            dy = path[-1+w][1] - self.bots[bot_id].loc[1]    
                            back_=2
                            # path.append(path[-1])
                            # self.bots[bot_id].set_path(path)
                            # continue                  
                        else :    
                            continue

                        if(dx!=0):
                                if self.map.is_walkable([path[-1][0],path[-1][1]+1]) and self._has_rob([path[-1][0],path[-1][1]+1])==0:   #x处碰撞且y方向有位置
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    #     # if bot_id == 2:
                                    #     #     debug.log('xxxxxxxxxxxxxxxxxxx')                
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                        
                                    #     # if bot_id == 2:
                                    #     #     debug.log('cccccccccccccccccc')                
                                    # elif self.map.is_walkable([path[-1][0],path[-1][1]+2]) and self._has_rob([path[-1][0],path[-1][1]+2])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1           
                                        # if bot_id == 2:
                                        #     debug.log('vvvvvvvvvvvvvvvvvvvvvvvv')       

                                    if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]+1,path[-1][1]+1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]+1,path__[1]+1]))                                        
                                        path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1
                                    elif self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]+1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]-1,path__[1]+1]))                                        
                                        path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1                                                       
                                    # else:
                                    
                                    else:
                                        path.append(np.array([path[-1][0],path[-1][1]+1]))  
                                        path.append(np.array([path[-1][0],path[-1][1]]))  
                                        path.append(path[-3])
                                        # path.append(np.array([path[-1][0],path[-1][1]+1]))  
                                        # path.append(path[-2])                                        
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1             
                                        self.avoid_buf[bot_id]=b.id
                                elif self.map.is_walkable([path[-1][0],path[-1][1]-1]) and self._has_rob([path[-1][0],path[-1][1]-1])==0:
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    #     # if bot_id == 2:
                                    #     #     debug.log('xxxxxxxxxxxxxxxxxxx')                
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                        
                                    #     # if bot_id == 2:
                                    #     #     debug.log('cccccccccccccccccc')                
                                    # elif self.map.is_walkable([path[-1][0],path[-1][1]-2]) and self._has_rob([path[-1][0],path[-1][1]-2])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1           
                                    #     # if bot_id == 2:
                                    #     #     debug.log('vvvvvvvvvvvvvvvvvvvvvvvv')       
                                    if self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]+1,path[-1][1]-1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]+1,path__[1]-1]))                                        
                                        path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1
                                    elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]-1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]-1,path__[1]-1]))                                        
                                        path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1                                                       
                                    else:
                                        path.append(np.array([path[-1][0],path[-1][1]-1]))  
                                        path.append(np.array([path[-1][0],path[-1][1]]))  
                                        path.append(path[-3])
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))  
                                        # path.append(path[-2])                                        
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1             
                                        self.avoid_buf[bot_id]=b.id

                                elif self.map.is_walkable([path[-1][0]+dx,path[-1][1]]) and self._has_rob([path[-1][0]+dx,path[-1][1]])==0 and back_==1:
                                    path.append(np.array([path[-1][0]+dx,path[-1][1]]))
                                    path.append(path[-2])
                                    self.bots[bot_id].set_path(path)
                                    avoid = 1                  
                                    self.avoid_buf[bot_id]=b.id                                  
                                else:
                                    avoid =2
                                    self.avoid_buf[bot_id]=b.id


                        elif dy !=0:
                            if self.map.is_walkable([path[-1][0]+1,path[-1][1]]) and self._has_rob([path[-1][0]+1,path[-1][1]]) ==0:  
                                # if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1]) ==0:
                                #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                #     path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                #     path.append(path[-4])
                                #     self.bots[bot_id].set_path(path)
                                #     avoid = 1
                                    
                                #     # if bot_id == 2:
                                #     #     debug.log('jjjjjjjjjjjjjjjjjjjjjiiiiiiiiiiiiiio')     
                                # elif self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1]) ==0:
                                #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                #     path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                #     path.append(path[-4])
                                #     self.bots[bot_id].set_path(path)
                                #     avoid = 1                        
                                #     # if bot_id == 2:
                                #     #     debug.log('jjjjjjjjjjjjjjjjjjjjjlllllll')     
                                # elif self.map.is_walkable([path[-1][0]+2,path[-1][1]]) and self._has_rob([path[-1][0]+2,path[-1][1]]) ==0:
                                #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                #     path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                #     path.append(path[-4])
                                #     self.bots[bot_id].set_path(path)
                                    # avoid = 1            
                                #     # if bot_id == 2:
                                #     #     debug.log('jjjjjjjjjjjjjjjjjjjjjkkkk')     
                                # else:
                                if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]+1,path[-1][1]+1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]+1,path__[1]+1]))                                        
                                        path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1
                                elif self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]-1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]+1,path__[1]-1]))                                        
                                        path.append(np.array([path[-1][0],path[-1][1]+1]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1                                                       
                                else:                                           
                                    path.append(np.array([path[-1][0]+1,path[-1][1]]))  
                                    path.append(np.array([path[-1][0],path[-1][1]]))  
                                    path.append(path[-3])
                                    # path.append(np.array([path[-1][0]+1,path[-1][1]]))  
                                    # path.append(path[-2])                                    
                                    self.bots[bot_id].set_path(path)
                                    avoid = 1                                                             
                                    self.avoid_buf[bot_id]=b.id
                                    # avoid =2
                            elif self.map.is_walkable([path[-1][0]-1,path[-1][1]]) and self._has_rob([path[-1][0]-1,path[-1][1]])==0:
                                # if self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0:
                                #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                #     path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                #     path.append(path[-4])
                                #     self.bots[bot_id].set_path(path)
                                #     avoid = 1
                                #     # if bot_id == 2:
                                #     #     debug.log('jjjjjjjjjjjjjjjjjjjjjhhhhhhhhh')     
                                # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1]):
                                #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                #     path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                #     path.append(path[-4])
                                #     self.bots[bot_id].set_path(path)
                                #     avoid = 1                        
                                #     # if bot_id == 2:
                                #     #     debug.log('jjjjjjjjjjjjjjjjjjjjjeeeeee')     
                                # elif self.map.is_walkable([path[-1][0]-2,path[-1][1]]) and self._has_rob([path[-1][0]-2,path[-1][1]])==0:
                                #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                #     path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                #     path.append(path[-4])
                                #     self.bots[bot_id].set_path(path)
                                #     avoid = 1      
                                #     # if bot_id == 2:
                                #     #     debug.log('jjjjjjjjjjjjjjjjjjjjwwwwwwwwwj')     
                                # else:

                                if self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]+1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]-1,path__[1]+1]))                                        
                                        path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1
                                elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]-1])))  ==1 and self.map_id!=2:
                                    # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                        path__ = path.pop()
                                        path.append(np.array([path__[0]-1,path__[1]-1]))                                        
                                        path.append(np.array([path[-1][0],path[-1][1]+1]))
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                        # path.append(self.bots[bot_id].loc)
                                        self.bots[bot_id].set_path(path)
                                        avoid = 1                                                       
                                else:                                
                                    path.append(np.array([path[-1][0]-1,path[-1][1]]))  
                                    path.append(np.array([path[-1][0],path[-1][1]]))  
                                    path.append(path[-3])
                                    # path.append(np.array([path[-1][0]-1,path[-1][1]]))  
                                    # path.append(path[-2])                                    
                                    self.bots[bot_id].set_path(path)
                                    avoid = 1                                                              
                                    self.avoid_buf[bot_id]=b.id          
                                    # avoid =2
                            elif self.map.is_walkable([path[-1][0],path[-1][1]+dy]) and self._has_rob([path[-1][0],path[-1][1]+dy])==0 and back_==1:
                                    path.append(np.array([path[-1][0],path[-1][1]+dy]))
                                    path.append(path[-2])
                                    self.bots[bot_id].set_path(path)
                                    avoid = 1                  
                                    self.avoid_buf[bot_id]=b.id
                            else:
                                avoid =2
                                self.avoid_buf[bot_id]=b.id
                                # if bot_id == 2:
                                #         debug.log('jjjjjjjjjjjjjjjjjjjjj')     

                    
##############################                                                           
                    elif dist ==2:  #同一目标点
                        # if (self.bots[bot_id].loc == path_buf[-i]).all() ==False or (path[-i] == self.bots[b.id].loc).all() ==False: 
                        #     continue
                        # debug.log(str(b.id)+'jjjj'+str(bot_id))           
                        if (path[-2] == path_buf[-2]).all() == False:      
                            continue

                        dx = path_buf[-2][0] - self.bots[bot_id].loc[0]
                        dy = path_buf[-2][1] - self.bots[bot_id].loc[1]           
                        if 1:
###################################################################################
                            if(dx!=0):
                                if self.map.is_walkable([path[-1][0],path[-1][1]+1]) and self._has_rob([path[-1][0],path[-1][1]+1])==0:   #x处碰撞且y方向有位置
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                        
                                    # elif self.map.is_walkable([path[-1][0],path[-1][1]+2]) and self._has_rob([path[-1][0],path[-1][1]+2])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                                                                  
                                    # else:
                                        # path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                        # path.append(np.array([path[-1][0],path[-1][1]]))
                                        # path.append(path[-3])
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]+1,path[-1][1]+1])))  ==1:
                                    # # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path__ = path.pop()
                                    #     path.append(np.array([path__[0]+1,path__[1]+1]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     # path.append(self.bots[bot_id].loc)
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]+1])))  ==1:
                                    # # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path__ = path.pop()
                                    #     path.append(np.array([path__[0]-1,path__[1]+1]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     # path.append(self.bots[bot_id].loc)
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                                                       
                                    # else:                                        
                                        path.append(np.array([path[-1][0],path[-1][1]+1]))                                        
                                        path.append(path[-2])                                        
                                        self.bots[bot_id].set_path(path)
                                        avoid =1
                                        self.avoid_buf[bot_id]=b.id
                                elif self.map.is_walkable([path[-1][0],path[-1][1]-1]) and self._has_rob([path[-1][0],path[-1][1]-1])==0:
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                        
                                    # elif self.map.is_walkable([path[-1][0],path[-1][1]-2]) and self._has_rob([path[-1][0],path[-1][1]-2])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1           
                                    # else:
                                        # path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                        # path.append(np.array([path[-1][0],path[-1][1]]))
                                        # path.append(path[-3])
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]+1,path[-1][1]-1])))  ==1:
                                    # # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path__ = path.pop()
                                    #     path.append(np.array([path__[0]+1,path__[1]-1]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     # path.append(self.bots[bot_id].loc)
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]-1])))  ==1:
                                    # # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path__ = path.pop()
                                    #     path.append(np.array([path__[0]-1,path__[1]-1]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     # path.append(self.bots[bot_id].loc)
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                     
                                    # else:                                        
                                        path.append(np.array([path[-1][0],path[-1][1]-1]))                                        
                                        path.append(path[-2])                                        
                                        self.bots[bot_id].set_path(path)
                                        avoid =1
                                        self.avoid_buf[bot_id]=b.id
                                elif self.map.is_walkable([path[-1][0]-dx,path[-1][1]]) and self._has_rob([path[-1][0]-dx,path[-1][1]])==0:
                                    # path.append(np.array([path[-1][0]-dx,path[-1][1]]))
                                   
                                    path.append(path[-1])
                                    # path.append(path[-2])
                                    self.bots[bot_id].set_path(path)
                                    avoid = 1       
                                    self.avoid_buf[bot_id]=b.id
                                else:
                                    avoid =3
                                    self.avoid_buf[bot_id]=b.id
                            elif dy !=0:
                                if self.map.is_walkable([path[-1][0]+1,path[-1][1]]) and self._has_rob([path[-1][0]+1,path[-1][1]]) ==0:  
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1]) ==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1]) ==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                        
                                    # elif self.map.is_walkable([path[-1][0]+2,path[-1][1]]) and self._has_rob([path[-1][0]+2,path[-1][1]]) ==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1       
                                    # if self.map.is_walkable([path[-1][0]+1,path[-1][1]+1]) and self._has_rob([path[-1][0]+1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]+1,path[-1][1]+1])))  ==1:
                                    #     # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #         path__ = path.pop()
                                    #         path.append(np.array([path__[0]+1,path__[1]+1]))                                        
                                    #         path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #         # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #         # path.append(self.bots[bot_id].loc)
                                    #         self.bots[bot_id].set_path(path)
                                    #         avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]+1,path[-1][1]-1]) and self._has_rob([path[-1][0]+1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]-1])))  ==1:
                                    #     # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #         path__ = path.pop()
                                    #         path.append(np.array([path__[0]+1,path__[1]-1]))                                        
                                    #         path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #         # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #         # path.append(self.bots[bot_id].loc)
                                    #         self.bots[bot_id].set_path(path)
                                    #         avoid = 1                                                       
                                    # # else:     


                                    # else:
                                        # path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                        # path.append(np.array([path[-1][0],path[-1][1]]))
                                        # path.append(path[-3])
                                        path.append(np.array([path[-1][0]+1,path[-1][1]]))                                        
                                        path.append(path[-2])                                    
                                        self.bots[bot_id].set_path(path)
                                        avoid =1                                        
                                        self.avoid_buf[bot_id]=b.id
                                        # avoid =3     
                                elif self.map.is_walkable([path[-1][0]-1,path[-1][1]]) and self._has_rob([path[-1][0]-1,path[-1][1]])==0:
                                    # if self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1]):
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                    #     path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #     path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1                        
                                    # elif self.map.is_walkable([path[-1][0]-2,path[-1][1]]) and self._has_rob([path[-1][0]-2,path[-1][1]])==0:
                                    #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                    #     path.append(np.array([path[-1][0]-1,path[-1][1]]))
                                    #     path.append(np.array([path[-1][0]+1,path[-1][1]]))
                                    #     path.append(path[-4])
                                    #     self.bots[bot_id].set_path(path)
                                    #     avoid = 1      
                                    # if self.map.is_walkable([path[-1][0]-1,path[-1][1]+1]) and self._has_rob([path[-1][0]-1,path[-1][1]+1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]+1])))  ==1:
                                    #     # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #         path__ = path.pop()
                                    #         path.append(np.array([path__[0]-1,path__[1]+1]))                                        
                                    #         path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #         # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #         # path.append(self.bots[bot_id].loc)
                                    #         self.bots[bot_id].set_path(path)
                                    #         avoid = 1
                                    # elif self.map.is_walkable([path[-1][0]-1,path[-1][1]-1]) and self._has_rob([path[-1][0]-1,path[-1][1]-1])==0 and np.sum(np.abs(path[-2] - np.array([path[-1][0]-1,path[-1][1]-1])))  ==1:
                                    #     # #     # path.append(np.array([path[-1][0],path[-1][1]]))
                                    #         path__ = path.pop()
                                    #         path.append(np.array([path__[0]-1,path__[1]-1]))                                        
                                    #         path.append(np.array([path[-1][0],path[-1][1]+1]))
                                    #         # path.append(np.array([path[-1][0],path[-1][1]-1]))
                                    #         # path.append(self.bots[bot_id].loc)
                                    #         self.bots[bot_id].set_path(path)
                                    #         avoid = 1                                                       
                                    # else:                                                                

                                    # else:
                                        # path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                        # path.append(np.array([path[-1][0],path[-1][1]]))
                                        # path.append(path[-3])
                                        path.append(np.array([path[-1][0]-1,path[-1][1]]))                                        
                                        path.append(path[-2])
                                        self.bots[bot_id].set_path(path)
                                        avoid =1                           
                                        self.avoid_buf[bot_id]=b.id                   
                                        # avoid =3
                                elif self.map.is_walkable([path[-1][0],path[-1][1]-dy]) and self._has_rob([path[-1][0],path[-1][1]-dy])==0:
                                    # path.append(np.array([path[-1][0],path[-1][1]-dy]))
                                    path.append(path[-1])
                                    # path.append(path[-2])
                                    self.bots[bot_id].set_path(path)
                                    avoid = 1                               
                                    self.avoid_buf[bot_id]=b.id
                                else:
                                    avoid =3
                                    self.avoid_buf[bot_id]=b.id
                           
                           
                            if avoid ==1:
                                continue


                    if avoid ==1:
                        continue

                    if avoid == 2 and len(path_buf)>=3 and (self.bots[bot_id].loc == path_buf[-1+w]).all() and (path[-1+w] == self.bots[b.id].loc).all():
                        dist = np.sum(np.abs(self.bots[bot_id].loc - path_buf[-3]))      
                        if back_ == 2:
                            path.append(path[-1])
                            self.bots[bot_id].set_path(path)
                        elif dist ==1:
                            # path.append(path_buf[-4])
                            path.append(path_buf[-3])
                            self.bots[bot_id].set_path(path)
                        else:
                            path.append(path[-1])
                            self.bots[bot_id].set_path(path)
                    if avoid == 3 and len(path_buf)>=4 and (self.bots[bot_id].loc[0]== self.bots[b.id].loc[0] or self.bots[bot_id].loc[1]== self.bots[b.id].loc[1]  ):
        
                        dist = np.sum(np.abs(self.bots[bot_id].loc - path_buf[-4]))      
                        if dist ==1:
                            # path.append(path_buf[-5])                                     
                            path.append(path_buf[-4])                
                            self.bots[bot_id].set_path(path)                      
                        else:
                            path.append(path[-1])
                            self.bots[bot_id].set_path(path)                              
      
        # for bot_id, bot in enumerate(self.bots): 
        #     # if(bot_id!=0):
        #     #     continue
        #     debug.log('xxrobot'+str(bot_id)+' pose'+str(self.bots[bot_id].loc))
        #     if len(self.bots[bot_id]._path)>0:
        #         for i in range(1, min(8,len(self.bots[bot_id]._path))+1):
        #             debug.log('xxrobot'+str(bot_id)+'  '+str(i)+'robot_path'+str(self.bots[bot_id]._path[-i]))        
        #         # if bot_id == 7:
        #         #     debug.log(str(len(self.bots[bot_id]._path))+'robot_path'+str(self.bots[bot_id]._path))        
        #     else:
        #         debug.log('xxrobot'+str(bot_id)+' nopath')

        for i in range(0,len(clear_buf)):
            bot = self.bots[clear_buf[i]]        
            path_ = self.bots[clear_buf[i]].get_path()
            if  len(path_)==2 and( path_[-1] == bot.loc).all() and( path_[-2] == bot.loc).all():
                self.bots[clear_buf[i]].set_path([])

        for i in range(0,len(clear_buf_)):
            bot = self.bots[clear_buf_[i]]        
            path_ = self.bots[clear_buf_[i]].get_path()
            if  len(path_)==2 and( path_[-1] == bot.loc).all() and( path_[-2] == bot.loc).all():
                self.bots[clear_buf_[i]].pop_path()    
        # debug.log(str(self.avoid_buf )+'robot'+str([self.avoid_buf[b.id]] )+' nopath'+str([self.avoid_buf[bot_id]]))           
    def _ctl_bots(self) -> None:
        """ 机器人状态运维，如自动沿路行走 """
        # 优先级高的晚走（低的要给他让路）
        for bot in self._bots_by_sup:# self.bots:  
            bot_id = bot.id
            # debug.log('_ctl_bots'+str(bot_id))                      
            # 没有路径/在重算路径, 退出
            if bot.no_path() or (10+bot_id) in self._bot_researching:
                continue 
            
            loc = bot.loc
            target = bot.next_path()
            d = target - loc
            if (loc == target).all(): # 到达目标点了就换新                
                target = self.bots[bot_id].pop_path()
                if not self.bots[bot_id].no_path():  # 拿取下一步
                    target = bot.next_path()
                    self._bot_last_update[bot_id] = self.frame
                    d = target - loc
                # else:
                #     continue
                    
            if not self.map.is_walkable(target):    # 不可走
                sign = '?'
                # 查找区块对应符号
                for k, v in map_class.items():
                    if v == self.map[target[1]][target[0]]:
                        sign = k
                        break
                raise IOError(f'Bot {bot_id} cannot walk to locate {target}, which is \'{sign}\' !')

            next_cmd = '-'
            if(d[0]>0):
                next_cmd = 'd'
            elif d[0]<0:
                next_cmd = 'a'
            elif d[1]>0:
                next_cmd = 's'
            elif d[1]<0 :
                next_cmd = 'w'
            next_dist = 0                

            # debug.log('robot'+str(bot_id)+' next_cmd:'+str(next_cmd))
                
            # 成功找到最佳下一步，进行执行
            if next_cmd != '-':   
                self._cmd(self.bots[bot_id]._move(next_cmd))
            
                # 最后一格，判断是否要执行get/pull
                if next_dist == 0 and bot.size_path() == 1:
                    next_loc = loc + directs[next_cmd]    # 下一步的位置
                    # 判断下一格的类型
                    # 1.空地，则说明可能是货物
                    if self.map[next_loc[1]][next_loc[0]] == map_class['.'] and self.bots[bot_id].goods==0:   
                        # 有货物，拿取
                        if find_loc_in_list(self.cargos.queue, next_loc) != -1:  
                            self._cmd(self.bots[bot_id].get())
                            self._add_next_frame_task(self._bot_verify, args=(bot_id, ACTION_BOT_GET, next_loc))
                            break
                    # 2. 泊口
                    elif self.map[next_loc[1]][next_loc[0]] == map_class['B'] and self.bots[bot_id].goods==1: 
                        self._cmd(self.bots[bot_id].pull())     
                        self._add_next_frame_task(self._bot_verify, args=(bot_id, ACTION_BOT_PULL, next_loc)) 
                        break
                    # N. 都不是，清空指令
                    self.bots[bot_id].set_path([])
 
    def _boat_get_cargo(self, boat_id):
        """ 
        船获得货物 
        @return:  0 - 成功
                 -1 - 目标不为泊口
                 -2 - 船不在泊口
        """
        boat = self.boats[boat_id]
        target_berth = boat.target
        
        # 目标不为泊口
        if target_berth == -1: 
            return -1
        
        # 船不在泊口
        if not boat.is_arrive():
            return -2
        
        # 泊口有物品，且有船在
        if self.berths[target_berth].count():
            last_size = self.boats[boat_id].free()  # 剩余空间
            objs = self.berths[target_berth].get_cargos(last_size) # 拿取货物
            self.boats[boat_id].get(objs)
            
        return 0              
        
    def _callback_bot_research(self, result):
        """ 机器人寻路的回调函数 """
        bot_id, path = result
        
        # if isinstance(path, list):
        #     debug.log(f'callback success:{bot_id}, {path[0]}')
        # else:
        #     debug.log(f'callback fail:{bot_id}, {path}')
            
        # 寻路成功
        if isinstance(path, list):
            path.pop()
            self.bots[bot_id].set_path(path)
            self._bot_timeout[bot_id] = 0   # 清空超时计数
        else:
            self._bot_timeout[bot_id] += 1  # 超时计数+1
            
        self._bot_last_update[bot_id] = self.frame
        self._bot_researching.remove(10+bot_id) # 删除标记
                     
    def _callback_dfs_init(self, masks):
        """ dfs初始化的回调函数（多进程） """
        dlen = len(masks)
        self.berths_dfs_map[-dlen:] = masks.copy()
        

if __name__ == "__main__":
    """ 测试寻路 """
    import os

    with open(os.path.abspath('./maps/map1.txt'), "r", encoding='utf-8') as f:  # 读取地图
        map_str = f.read()  # 读取文本
        map_str = map_str.split('\n')
        map, _ = str2map(map_str)  # 解析地图

        map = np.array([
            [1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1],
            [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1],
            [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
            [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
            [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
        ])
        map = Map(map)
        path = map.a_star([193, 70], [117, 137])
        # path = map.a_star([0, 0], [5, 0])
        print(path)
        for x, y in path:
            map[y][x] = 8
        print(map)
else:
    console = Driver()
