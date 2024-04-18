"""
地图相关代码
"""


import numpy as np
from tree import QuadTree
import debug


map_size = [200, 200]  # 地图尺寸[h, w]
map_class = {  # 地图标识对应编号(正数则为可行走区域)
    'B': 0,  # 泊位位置4*4
    '.': 1,  # 空地
    '*': -1,  # 海洋
    '#': -2,  # 障碍
    'A': -3,  # 机器人起始位置
}
MAP_BERTH = map_class['B']  # 泊位位置4*4
MAP_ROAD = map_class['.']  # 空地
MAP_SEA = map_class['*']  # 海洋
MAP_OBSTACLE = map_class['#']  # 障碍

directs = {  # 机器人移动方向对应表
    'w': np.array([0, -1]),
    'a': np.array([-1, 0]),
    's': np.array([0,  1]),
    'd': np.array([ 1, 0]),
}

HAST_VAL = 400 # 计算哈希的乘数


class Map(np.ndarray):
    """ 地图 """
    h, w = 0, 0

    def __new__(cls, input_array=np.zeros((1, 1)), info=None):
        # Input array is an already formed ndarray instance
        # We first cast to be our class type
        obj = np.asarray(input_array).view(cls)
        # add the new attribute to the created instance
        obj.info = info
        # Finally, we must return the newly created object:
        return obj

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.info = getattr(obj, 'info', None)

    def __init__(self, map_array=np.zeros((1, 1))) -> None:
        """
        @param map_array: 地图数组
        """
        super().__init__()
        self.h, self.w = self.shape  # 地图形状

    def is_class(self, loc, c: str):
        """ 
        某个点是否为某个类 
        @param c: '.' 或 '.A'
        """
        val = self[loc[1]][loc[0]]
        for sign in c:
            if val == map_class[sign]:
                return True
        return False

    def at(self, loc: list):
        """ 访问地图数据 """
        return self[loc[1]][loc[0]]

    def is_board(self, loc):
        """ 坐标是否在地图外 """
        if not (-1 < loc[0] < self.w and -1 < loc[1] < self.h):
            return True
        return False

    def is_walkable(self, loc):
        """ 坐标是否可以移动 """
        if self.is_board(loc):
            return False
        return self[loc[1]][loc[0]] >= 0

    def get_sign(self, loc:np.array):
        """ 获取地图位置的符号 """
        val = self.at(loc)
        for k, v in map_class.items():
            if v==val:
                return k
    


def path_search_a_star(map:Map, start, target, max_step=40000, obstacles=[]):
    import heapq
    """
    A*寻路算法
    Args:
        map (Map): 地图数组
        start (_type_):   起始点
        target (_type_):  目标点
        obstacles (_type_):  障碍物坐标
        
    Returns:
        list: 路径
        int:  -1 - 起始/目标点不为路
              -2 - 寻路失败/距离过远
    """
    class Node(object):
        def __init__(self, loc: np.ndarray, g, h, father):
            self.loc = loc  # 位置
            self.g = g  # 起始节点到当前节点的距离代价
            self.h = h  # 当前节点到目标节点的估计代价
            self.f = g + h  # 移动代价
            self.father = father

        def __lt__(self, other):
            return self.f < other.f
        
    obsts = []
    if len(obstacles):
        if not isinstance(obstacles[0], int):
            obsts = [loc2hash(i) for i in obstacles]    # 障碍的哈希表
        else:
            obsts = obstacles

    directs = np.array([[0, -1], [0, 1], [-1, 0], [1, 0]])  # 上，下，左，右

    if not (map.is_walkable(start) and map.is_walkable(target)):
        return -1

    start = np.array(start)
    target = np.array(target)

    startNode = Node(start, 0, np.sum(np.abs(target - start)), None)
    openList = []  # 待遍历的列表
    lockList = set()  # 存放已遍历位置的哈希值
    currNode = startNode
    lockList.add(loc2hash(currNode.loc))
    heapq.heappush(openList, startNode)

    while True:
        # 寻路失败
        if not len(openList):
            return -2

        # 查找openlist中val最小的节点，作为currNode
        currNode = heapq.heappop(openList)
        if (currNode.loc == target).all():  # 为终点
            break

        # 寻找周围的可行点
        for d in directs:
            next_loc = currNode.loc + d
            next_hash = loc2hash(next_loc)
            if map.is_walkable(next_loc) and \
                (next_hash not in lockList) and \
                (next_hash not in obsts):
                g = currNode.g+1
                if g <= max_step:   # 限制步长                    
                    next_node = Node(next_loc, g, np.sum(np.abs(target - next_loc)), currNode)
                    # 添加/修改记录
                    openList.append(next_node)
                    lockList.add(loc2hash(next_node.loc))

    # 根据结果索引
    result = []
    while (currNode.father != None):
        result.append(currNode.loc)
        currNode = currNode.father
    result.append(currNode.loc)
    return result[::-1]


def path_search_tree(quad_tree:QuadTree, map:Map, start, target, max_step=40000):
    """
    关键点寻路算法
    Args:
        map (Map): 地图数组
        start (_type_):   起始点
        target (_type_):  目标点
        
    Returns:
        list: 路径
        int:  -1 - 起始/目标点不为路
              -2 - 寻路失败 
    """    
    if not (map.is_walkable(start) and map.is_walkable(target)):
        return -1
    
    # 找start和target全方向最近的关键点，返回[过去的代价距离,最近关键的id]
    # Attention! 这里loc把x,y反过来是因为坐标系不同，tree为map[x][y]，而driver为map[y][x]
    start_cost_distance, start_cost_id = quad_tree.Search(start[1],start[0], max_step,0)
    end_cost_distance, end_cost_id = quad_tree.Search(target[1],target[0], max_step,0)
    
    # 寻路失败
    if start_cost_distance==9999 or end_cost_distance==9999:    
        return -2
    
    # 找这两个关键点间的最短路径
    debug.log('node locs: ', quad_tree.id_to_keypoint[start_cost_id], quad_tree.id_to_keypoint[end_cost_distance])
    debug.log('ids: ', start_cost_id, end_cost_id)
    list_clost_id_, cost_distance = quad_tree.shortest_path(start_cost_id, end_cost_id)
    
    # 寻路失败
    if list_clost_id_ is None:  
        return -2
    
    path = []
    for i in range(len(list_clost_id_)):    # 把节点转化为路径
        loc = quad_tree.id_to_keypoint[list_clost_id_[i]]
        path.append(np.array([loc[1], loc[0]]))
    return path


def path_search_dfs(dfs_map:np.ndarray, start, obstacles=[]):
    """
    dfs寻路算法，计算start位置到泊口的路径
    Args:
        dfs_map (np.ndarray): 泊口对应的dfs地图
        start (_type_):   出发点
        
    Returns:
        list: 路径
        int:  -1 - 出发点不为路
              -2 - 寻路失败
    """    
    start = np.array(start)
    obsts = []
    if len(obstacles):
        if not isinstance(obstacles[0], int):
            obsts = [loc2hash(i) for i in obstacles]    # 障碍的哈希表
        else:
            obsts = obstacles
            
    
    h, w = dfs_map.shape
    # 超出边界 or 不可移动
    if not (-1 < start[0] < w and -1 < start[1] < h) or \
        dfs_map[start[1]][start[0]] < 0:
        return -1
    
    path = []
    walked = set()  # 已走过的路
    path.append(start)
    now_dist = dfs_map[start[1]][start[0]]
    
    while now_dist > 0:
        directions = [] # [距离，位置]
        
        for dirct in directs.values():
            next_loc = path[-1] + dirct

            if not (-1 < next_loc[0] < w and -1 < next_loc[1] < h):
                continue

            next_dist = dfs_map[next_loc[1]][next_loc[0]]
            
            next_hash = loc2hash(next_loc)
            
            # 不可行走 / 障碍物
            if next_dist<0 or \
                next_hash in obsts or \
                next_hash in walked:
                continue
            
            directions.append([next_dist, next_loc, next_hash])
            
        if len(directions) < 1: # 没有路径
            return -2
            
        # 按距离排序
        directions.sort(key=lambda x: x[0])
            
        # 选取距离最低的作为下降方向
        now_dist = directions[0][0]
        next_loc = directions[0][1]
        next_hash = directions[0][2]
        
        # 找不到，在一个点循环了
        # debug.log(len(path), path[-1], next_loc )
        if len(path)>1 and (path[-1]==next_loc).all():
            return -2
        
        path.append(next_loc)
        walked.add(next_hash)

    
    return path
        


def dfs_genMap(map:Map, locs:list):
    """ 
    dfs 计算地图上相对某个点的梯度距离
    @param locs: 位置列表(多个，这样会返回多个矩阵，避免多次计算mask_raw)
    
    @return
    >=0: 与该点的梯度距离
    -1: 未搜索到
    -2: 无法进入（障碍物）
    """ 
    mask_raw = np.where(map>=0, -1, -2)  
    map = Map(map) 
    
    masks = []
    for loc in locs:
        mask = mask_raw.copy()
        waitList = [loc] # 等待计算的点
        dist_now = 0
        
        while len(waitList) > 0:
            next_waitList = []
            for wait in waitList:
                # 当前点赋值
                mask[wait[1]][wait[0]] = dist_now
                # 寻找周围的点
                for dir in directs.values():
                    next = wait + dir
                    if not map.is_board(next) and mask[next[1]][next[0]] == -1:    # 可以走的未搜索区块
                        next_waitList.append(next)
                        mask[next[1]][next[0]] = -3 # 等下一帧算，避免一个点重复加入
            dist_now += 1
            waitList = next_waitList       
        masks.append(mask)
    return masks


def str2map(map_str: str):
    """
    从字符串解析地图信息
    @return : 地图对象，机器人坐标列表
    """
    bot_locs = []  # 机器人初始位置[x, y]
    map_int = []  # 位置数组
    for y, line in enumerate(map_str):
        map_line = []
        for x, char in enumerate(line):
            if char == 'A':  # 找到机器人，记录初始坐标
                bot_locs.append([x, y])
                map_line.append(map_class['.'])
            else:
                map_line.append(map_class[char])
        map_int.append(map_line)

    return Map(np.array(map_int, dtype=np.int8)), bot_locs

def loc2hash(loc):
    """ 计算位置的哈希值 """
    return loc[0] * HAST_VAL + loc[1]

def hash2loc(hash:int):
    x = int(hash / HAST_VAL)
    y = int(hash % HAST_VAL)
    return np.array([x, y])

def path_simplify(path):
    """ 
    路径简化 
    ps: 目前的思路就是把相同方向的去掉
    """
    output = []
    
    last_dir = [0, 0]
    for i in range(0, len(path)-1):
        now_dir = [path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]]
        
        # 方向不一致时则添加
        if not now_dir == last_dir:
            output.append(path[i+1])
            last_dir = now_dir
    # 终点还是要的
    if not (output[-1] == path[-1]).all():
        output.append(path[-1])
    return output

def berth_locs_by_score(berth_loc, map:Map):
    """ 
    对泊口的位置进行评分，并按递减顺序排列 
    @param berth_loc: 泊口的位置（左上角）
    @param map: 地图文件
    @return list: [loc, loc, loc]
    """
    sample_area = [[i, j] for j in range(-1, 2) for i in range(-1, 2)]
    sample_area.remove([0,0])
    h, w = map.shape
    
    scores = []
    for i in range(4):
        for j in range(4):
            score = 0  # 周围每个点的总分
            # 基准点
            x = berth_loc[0] + j
            y = berth_loc[1] + i
            # 对周围采样
            for dx, dy in sample_area:     
                x_ = x+dx
                y_ = y+dy
                
                # 边界里
                if 0 < x_ < w and 0 < y_ < h:
                    sign = map[y_][x_]  # 该点的值
                    if sign == map_class['B']:      # 泊口
                        score += 0
                    elif sign == map_class['.']:    # 空地
                        score += 2
                    else:                           # 障碍
                        score -= 1
                # 超出边界
                else:
                    score -= 1
            # 记录得分
            scores.append([score, [x,y]])
    # 根据得分排序
    scores.sort(key=lambda x:x[0], reverse=True)
    
    # 输出结果
    result = [loc for _, loc in scores]
    return np.array(result)
                    
def cal_nearby_area(map:Map, loc:np.ndarray, dist:int, bots=None):
    """ 
    获取坐标附近N格距离内的可行区域数 
    @param map: 地图信息
    @param loc: 要查找的坐标
    @param dist: N格距离内
    @param bots(optional): 输入机器人列表，会自动视为障碍物
    @return: =0: 周围没有
             >0: N格距离内的可行区域数
    """
    # if not isinstance(map, Map):
    map = Map(map) 
    
    waitList = [loc] # 等待计算的点
    walked = set()  # 已经走过的路(hash值)
    walked.add(loc2hash(loc))
    dist_now = 0
    
    # 将机器人视为障碍物
    bots_loc = set()
    if bots is not None:
        for b in bots:
            bots_loc.add(loc2hash(b.loc))
    
    # 开始计算
    while len(waitList) > 0 and dist_now < dist:
        next_waitList = []
        for wait in waitList:
            # 寻找周围的点
            for dir in directs.values():
                next = wait + dir
                next_hash = loc2hash(next)
                if not map.is_board(next) and next_hash not in walked and next_hash not in bots_loc:    # 可以走的未搜索区块
                    next_waitList.append(next)
                    walked.add(next_hash)
        dist_now += 1
        waitList = next_waitList     

    result = len(walked) - 1
    return result

