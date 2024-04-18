import sys
import os
sys.path.append(os.getcwd())
import random
import datetime
#### 调试用，提交时请注释掉！ ####
import debug    # debug库

import numpy as np

n = 200
robot_num = 10
berth_num = 10
N = 210



class Robot:
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby

robot = [Robot() for _ in range(robot_num + 10)]

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed

berth = [Berth() for _ in range(berth_num + 10)]

class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status

boat = [Boat() for _ in range(10)]


money = 0
boat_capacity = 0
id = 0
ch = []
gds = [[0 for _ in range(N)] for _ in range(N)]






###########################################################################
###########################################################################
###########################################################################






class QNode:
    def __init__(self,x,y,id):
        self.x=x
        self.y=y
        self.DL=None
        self.DR=None
        self.UL=None
        self.UR=None
        self.id=id

class QuadTree:

    def __init__(self):
        self.root=None
        self.near_point_dis = 50
        self.near_point=[]
        self.near_point_id=0
        self.dri_map = []       
        #dri_map[i][j] 第i+1行j+1列是否为可行驶区域 是为ture 否则为false
        self.feature_map = [] 
        #feature_map[i][j] 第i+1行j+1列的特征 为66时为关键点
        self.id_to_keypoint = []
        #id_to_keypoint[i][0] 表示第i个关键点的行 id_to_keypoint[i][1] 表示第i个关键点的列
        self.id_to_nearkeypoinidt = [] 
        #id_to_nearkeypoinidt[i][0][0] 第i个关键点的右下方最近关键点的距离代价
        #id_to_nearkeypoinidt[i][0][1] 第i个关键点的右下方最近关键点的id
        #id_to_nearkeypoinidt[i][1] 第i个关键点的右上方最近关键点
        #id_to_nearkeypoinidt[i][2] 第i个关键点的左上方最近关键点
        #id_to_nearkeypoinidt[i][3] 第i个关键点的左下方最近关键点

        berth_goto_keypoint_dis = []        #没写
        # berth_goto_keypoint_cost = []#没写
        keypoint_goto_berth = []#没写

    def init_Dijkstra(self,graph):
        self.graph = graph      # 邻接表
        # self.start = start      # 起点
        # self.goal = goal        # 终点
        self.open_list = {}     # open 表
        self.closed_list = {}   # closed 表
        # self.open_list[start] = 0.0     # 将起点放入 open_list 中
        # self.parent = {start: None}     # 存储节点的父子关系。键为子节点，值为父节点。方便做最后路径的回溯
        self.min_dis = None             # 最短路径的长度

    def shortest_path(self,start,goal):
        # debug.log(str('wwwwwwww'))
        self.start = start      # 起点
        self.goal = goal        # 终点
        self.open_list.clear()
        self.closed_list.clear()
        self.parent = {}
        self.open_list[start] = 0.0     # 将起点放入 open_list 中
        self.parent = {start: None}     # 存储节点的父子关系。键为子节点，值为父节点。方便做最后路径的回溯
        self.closed_list = {}
        shortest_path = []
        if(start ==goal):
            shortest_path = [self.goal]
            shortest_path.append(self.start)
            # debug.log('start'+str(start)+'goal'+str(goal)+'shortest_path'+str(shortest_path[::-1]))
            return shortest_path, 0
        # debug.log(str('wwwwwwww'))
        while True:
            # debug.log('len(self.open_list)'+str(len(self.open_list)))
            if len(self.open_list) < 1:
            # if self.open_list is None:
                debug.log(str('搜索失败， 结束！'))
                # print('搜索失败， 结束！')
                return None, None
            # debug.log('open_list', str(self.open_list))
            distance, min_node = min(zip(self.open_list.values(), self.open_list.keys()))      # 取出距离最小的节点
            # debug.log('open_list', str(min_node))
            self.open_list.pop(min_node)                                                       # 将其从 open_list 中去除

            self.closed_list[min_node] = distance                  # 将节点加入 closed_list 中
            # debug.log('yy')
            if min_node == self.goal:                              # 如果节点为终点
                self.min_dis = distance
                shortest_path = [self.goal]                        # 记录从终点回溯的路径
                father_node = self.parent[self.goal]
                while father_node != self.start:
                    shortest_path.append(father_node)
                    father_node = self.parent[father_node]
                    # debug.log('father_node'+str(father_node))
                    # debug.log('parent'+str(self.parent))                    
                shortest_path.append(self.start)
                # debug.log('start'+str(start)+'goal'+str(goal)+'shortest_path'+str(shortest_path[::-1]))

                return shortest_path[::-1], self.min_dis			# 返回最短路径和最短路径长度
            # debug.log('xx')
            for node in self.graph[min_node].keys():               # 遍历当前节点的邻接节点
                if node not in self.closed_list.keys():            # 邻接节点不在 closed_list 中
                    if node in self.open_list.keys():              # 如果节点在 open_list 中
                        if self.graph[min_node][node] + distance < self.open_list[node]:
                            self.open_list[node] = distance + self.graph[min_node][node]         # 更新节点的值
                            self.parent[node] = min_node           # 更新继承关系
                    else:                                          # 如果节点不在 open_list 中
                        self.open_list[node] = distance + self.graph[min_node][node]             # 计算节点的值，并加入 open_list 中
                        self.parent[node] = min_node               # 更新继承关系

    def shortest_path_(self,start,target):
        start_cost_distance, start_cost_id = self.Search(start[0],start[1], 50,0)
        # debug.log('start_cost_id',start_cost_id)               
        end_cost_distance, end_cost_id = self.Search(target[0],target[1], 50,0)
        # debug.log('end_cost_id',end_cost_id)              
        if start_cost_distance==9999 or end_cost_distance==9999:    
            return -2
        list_clost_id_, cost_distance = self.shortest_path(start_cost_id, end_cost_id)
        if list_clost_id_ is None:  
            return -2
        # for i in range(0, len(list_clost_id_)):
            # debug.log('key point'+str(self.id_to_keypoint[list_clost_id_[i]]))

        # debug.log('start'+str(start))
        # debug.log('target'+str(target))        
        path = []
        path.append(np.array([start[0], start[1]]))
        buf_x =  start[0]
        buf_y =  start[1]
        count_distan = 0  
 
        current_id = list_clost_id_[0]


        # dx = self.id_to_keypoint[current_id][1] - buf_x_
        # dy = self.id_to_keypoint[current_id][0] - buf_y_
        
        dx = self.id_to_keypoint[current_id][1] - buf_x
        dy = self.id_to_keypoint[current_id][0] - buf_y
        while((dx!=0 or dy !=0)):
            dx = self.id_to_keypoint[current_id][1] - buf_x
            dy = self.id_to_keypoint[current_id][0] - buf_y
            count_distan+=1             
            # if dx>0 and self.dri_map[buf_y][buf_x+1]== True and count_distan%2==0:
            #     buf_x+=1
            # elif dx<0 and self.dri_map[buf_y][buf_x-1]== True and count_distan%2==0:  
            #     buf_x-=1
            # elif dy>0 and self.dri_map[buf_y+1][buf_x]== True and count_distan%2:
            #     buf_y+=1
            # elif dy<0 and self.dri_map[buf_y-1][buf_x]== True and count_distan%2:  
            #     buf_y-=1
            if dx>0 and self.dri_map[buf_y][buf_x+1]== True :
                buf_x+=1
            elif dx<0 and self.dri_map[buf_y][buf_x-1]== True :  
                buf_x-=1
            elif dy>0 and self.dri_map[buf_y+1][buf_x]== True:
                buf_y+=1
            elif dy<0 and self.dri_map[buf_y-1][buf_x]== True:  
                buf_y-=1
            else:
                debug.log('shortest_path_00')                        
            dx = self.id_to_keypoint[current_id][1] - buf_x
            dy = self.id_to_keypoint[current_id][0] - buf_y     
            path.append(np.array([buf_x, buf_y]))        


        # debug.log('path111  '+str(path))
    #    # debug.log('start1')        
    #     for i in range(0, len(list_clost_id_)):
    #         debug.log('way'+str(self.id_to_keypoint[list_clost_id_[i]]))
    #     for i in range(0, len(self.id_to_keypoint)):
    #         buf1 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,1)
    #         buf2 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,2)
    #         buf3 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,3)        
    #         buf4 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,4)
    #         self.id_to_nearkeypoinidt.append([])
    #         self.id_to_nearkeypoinidt[i].append([buf1,buf2,buf3,buf4])
    #     # debug.log('xxx')




        for i in range(1, len(list_clost_id_)):
            # debug.log('list_clost_id_[i]'+str(list_clost_id_[i]))
            # debug.log('self.id_to_nearkeypoinidt[current_id][i]'+str(self.id_to_nearkeypoinidt[current_id]))           
            if list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][0][1] or list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][1][1] or list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][2][1] or list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][3][1]:
                dx = self.id_to_keypoint[list_clost_id_[i]][1] - buf_x
                dy = self.id_to_keypoint[list_clost_id_[i]][0] - buf_y
                while((dx!=0 or dy !=0)):
                    dx = self.id_to_keypoint[list_clost_id_[i]][1] - buf_x
                    dy = self.id_to_keypoint[list_clost_id_[i]][0] - buf_y
                    count_distan+=1
                    # debug.log('buf_x'+str(buf_x)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][1]))  
                    # debug.log('buf_y'+str(buf_y)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][0]))  
                    # if dx>0 and self.dri_map[buf_y][buf_x+1]== True and count_distan%2==0:
                    #     buf_x+=1
                    # elif dx<0 and self.dri_map[buf_y][buf_x-1]== True and count_distan%2==0:  
                    #     buf_x-=1
                    # elif dy>0 and self.dri_map[buf_y+1][buf_x]== True and count_distan%2:
                    #     buf_y+=1
                    # elif dy<0 and self.dri_map[buf_y-1][buf_x]== True and count_distan%2:  
                    #     buf_y-=1
                    if dx>0 and self.dri_map[buf_y][buf_x+1]== True :
                        buf_x+=1
                    elif dx<0 and self.dri_map[buf_y][buf_x-1]== True :  
                        buf_x-=1
                    elif dy>0 and self.dri_map[buf_y+1][buf_x]== True:
                        buf_y+=1
                    elif dy<0 and self.dri_map[buf_y-1][buf_x]== True:  
                        buf_y-=1
                    else:
                        debug.log('shortest_path_11')        
                    dx = self.id_to_keypoint[list_clost_id_[i]][1] - buf_x
                    dy = self.id_to_keypoint[list_clost_id_[i]][0] - buf_y       
                    path.append(np.array([buf_x, buf_y]))
            else:
                buf_x_ = self.id_to_keypoint[list_clost_id_[i]][1]
                buf_y_ = self.id_to_keypoint[list_clost_id_[i]][0]
                dx = self.id_to_keypoint[current_id][1] - buf_x_
                dy = self.id_to_keypoint[current_id][0] - buf_y_
                path_ = []
                while((dx!=0 or dy !=0)):
                    dx = self.id_to_keypoint[current_id][1] - buf_x_
                    dy = self.id_to_keypoint[current_id][0] - buf_y_
                    count_distan+=1
                    # debug.log('buf_x'+str(buf_x)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][1]))  
                    # debug.log('buf_y'+str(buf_y)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][0]))  
                    # if dx>0 and self.dri_map[buf_y_][buf_x_+1]== True and count_distan%2==0:
                    #     buf_x_+=1
                    # elif dx<0 and self.dri_map[buf_y_][buf_x_-1]== True and count_distan%2==0:  
                    #     buf_x_-=1
                    # elif dy>0 and self.dri_map[buf_y_+1][buf_x_]== True and count_distan%2:
                    #     buf_y_+=1
                    # elif dy<0 and self.dri_map[buf_y_-1][buf_x_]== True and count_distan%2:  
                    #     buf_y_-=1
                    if dx>0 and self.dri_map[buf_y_][buf_x_+1]== True :
                        buf_x_+=1
                    elif dx<0 and self.dri_map[buf_y_][buf_x_-1]== True :  
                        buf_x_-=1
                    elif dy>0 and self.dri_map[buf_y_+1][buf_x_]== True:
                        buf_y_+=1
                    elif dy<0 and self.dri_map[buf_y_-1][buf_x_]== True:  
                        buf_y_-=1
                    else:
                        debug.log('shortest_path_22')        
                    dx = self.id_to_keypoint[current_id][1] - buf_x_
                    dy = self.id_to_keypoint[current_id][0] - buf_y_
                    path_.append(np.array([buf_x_, buf_y_]))
                buf_x = self.id_to_keypoint[list_clost_id_[i]][1]
                buf_y = self.id_to_keypoint[list_clost_id_[i]][0]                
                while len(path_)!=0:
                    path.append(path_.pop())
            current_id = list_clost_id_[i]


        # debug.log('path222  '+str(path))

        # debug.log('start2')        
        buf_x_ = target[0]
        buf_y_ = target[1]       
        dx = buf_x - buf_x_
        dy = buf_y - buf_y_
        path_ = []        
        while((dx!=0 or dy !=0)):
            count_distan+=1             
            if dx>0 and self.dri_map[buf_y_][buf_x_+1]== True:
                buf_x_+=1
            elif dx<0 and self.dri_map[buf_y_][buf_x_-1]== True: 
                buf_x_-=1
            elif dy>0 and self.dri_map[buf_y_+1][buf_x_]== True:
                buf_y_+=1
            elif dy<0 and self.dri_map[buf_y_-1][buf_x_]== True:  
                buf_y_-=1
            else:
                debug.log('shortest_path_33')        
            dx = buf_x - buf_x_
            dy = buf_y - buf_y_            
            path_.append(np.array([buf_x_, buf_y_]))
        while len(path_)!=0:
            path.append(path_.pop())
        # path.append(np.array([buf_x, buf_y]))
        # debug.log('path33333'+str(path))


        
        return path


    def shortest_path_keypoint_id_(self,start,target):
        start_cost_distance, start_cost_id = self.Search(start[0],start[1], 50,0)
        # debug.log('start_cost_id',start_cost_id)               
        end_cost_distance, end_cost_id = self.Search(target[0],target[1], 50,0)
        # debug.log('end_cost_id',end_cost_id)              
        if start_cost_distance==9999 or end_cost_distance==9999:    
            return -2
        list_clost_id_, cost_distance = self.shortest_path(start_cost_id, end_cost_id)
        if list_clost_id_ is None:  
            return -2
        # for i in range(0, len(list_clost_id_)):
            # debug.log('key point'+str(self.id_to_keypoint[list_clost_id_[i]]))

        # debug.log('start'+str(start))
        # debug.log('target'+str(target))        
        path = []
        path.append(np.array([start[0], start[1]]))
        buf_x =  start[0]
        buf_y =  start[1]
        count_distan = 0  
 
        current_id = list_clost_id_[0]


        # dx = self.id_to_keypoint[current_id][1] - buf_x_
        # dy = self.id_to_keypoint[current_id][0] - buf_y_
        
        dx = self.id_to_keypoint[current_id][1] - buf_x
        dy = self.id_to_keypoint[current_id][0] - buf_y
        while((dx!=0 or dy !=0)):
            dx = self.id_to_keypoint[current_id][1] - buf_x
            dy = self.id_to_keypoint[current_id][0] - buf_y
            count_distan+=1             
            # if dx>0 and self.dri_map[buf_y][buf_x+1]== True and count_distan%2==0:
            #     buf_x+=1
            # elif dx<0 and self.dri_map[buf_y][buf_x-1]== True and count_distan%2==0:  
            #     buf_x-=1
            # elif dy>0 and self.dri_map[buf_y+1][buf_x]== True and count_distan%2:
            #     buf_y+=1
            # elif dy<0 and self.dri_map[buf_y-1][buf_x]== True and count_distan%2:  
            #     buf_y-=1
            if dx>0 and self.dri_map[buf_y][buf_x+1]== True :
                buf_x+=1
            elif dx<0 and self.dri_map[buf_y][buf_x-1]== True :  
                buf_x-=1
            elif dy>0 and self.dri_map[buf_y+1][buf_x]== True:
                buf_y+=1
            elif dy<0 and self.dri_map[buf_y-1][buf_x]== True:  
                buf_y-=1
            else:
                debug.log('shortest_path_00')                        
            dx = self.id_to_keypoint[current_id][1] - buf_x
            dy = self.id_to_keypoint[current_id][0] - buf_y     
            path.append(np.array([buf_x, buf_y]))        


        # debug.log('path111  '+str(path))
    #    # debug.log('start1')        
    #     for i in range(0, len(list_clost_id_)):
    #         debug.log('way'+str(self.id_to_keypoint[list_clost_id_[i]]))
    #     for i in range(0, len(self.id_to_keypoint)):
    #         buf1 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,1)
    #         buf2 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,2)
    #         buf3 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,3)        
    #         buf4 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,4)
    #         self.id_to_nearkeypoinidt.append([])
    #         self.id_to_nearkeypoinidt[i].append([buf1,buf2,buf3,buf4])
    #     # debug.log('xxx')




        for i in range(1, len(list_clost_id_)):
            # debug.log('list_clost_id_[i]'+str(list_clost_id_[i]))
            # debug.log('self.id_to_nearkeypoinidt[current_id][i]'+str(self.id_to_nearkeypoinidt[current_id]))           
            if list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][0][1] or list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][1][1] or list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][2][1] or list_clost_id_[i] == self.id_to_nearkeypoinidt[current_id][0][3][1]:
                dx = self.id_to_keypoint[list_clost_id_[i]][1] - buf_x
                dy = self.id_to_keypoint[list_clost_id_[i]][0] - buf_y
                while((dx!=0 or dy !=0)):
                    dx = self.id_to_keypoint[list_clost_id_[i]][1] - buf_x
                    dy = self.id_to_keypoint[list_clost_id_[i]][0] - buf_y
                    count_distan+=1
                    # debug.log('buf_x'+str(buf_x)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][1]))  
                    # debug.log('buf_y'+str(buf_y)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][0]))  
                    # if dx>0 and self.dri_map[buf_y][buf_x+1]== True and count_distan%2==0:
                    #     buf_x+=1
                    # elif dx<0 and self.dri_map[buf_y][buf_x-1]== True and count_distan%2==0:  
                    #     buf_x-=1
                    # elif dy>0 and self.dri_map[buf_y+1][buf_x]== True and count_distan%2:
                    #     buf_y+=1
                    # elif dy<0 and self.dri_map[buf_y-1][buf_x]== True and count_distan%2:  
                    #     buf_y-=1
                    if dx>0 and self.dri_map[buf_y][buf_x+1]== True :
                        buf_x+=1
                    elif dx<0 and self.dri_map[buf_y][buf_x-1]== True :  
                        buf_x-=1
                    elif dy>0 and self.dri_map[buf_y+1][buf_x]== True:
                        buf_y+=1
                    elif dy<0 and self.dri_map[buf_y-1][buf_x]== True:  
                        buf_y-=1
                    else:
                        debug.log('shortest_path_11')        
                    dx = self.id_to_keypoint[list_clost_id_[i]][1] - buf_x
                    dy = self.id_to_keypoint[list_clost_id_[i]][0] - buf_y       
                    path.append(np.array([buf_x, buf_y]))
            else:
                buf_x_ = self.id_to_keypoint[list_clost_id_[i]][1]
                buf_y_ = self.id_to_keypoint[list_clost_id_[i]][0]
                dx = self.id_to_keypoint[current_id][1] - buf_x_
                dy = self.id_to_keypoint[current_id][0] - buf_y_
                path_ = []
                while((dx!=0 or dy !=0)):
                    dx = self.id_to_keypoint[current_id][1] - buf_x_
                    dy = self.id_to_keypoint[current_id][0] - buf_y_
                    count_distan+=1
                    # debug.log('buf_x'+str(buf_x)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][1]))  
                    # debug.log('buf_y'+str(buf_y)+'goal'+str( self.id_to_keypoint[list_clost_id_[i]][0]))  
                    # if dx>0 and self.dri_map[buf_y_][buf_x_+1]== True and count_distan%2==0:
                    #     buf_x_+=1
                    # elif dx<0 and self.dri_map[buf_y_][buf_x_-1]== True and count_distan%2==0:  
                    #     buf_x_-=1
                    # elif dy>0 and self.dri_map[buf_y_+1][buf_x_]== True and count_distan%2:
                    #     buf_y_+=1
                    # elif dy<0 and self.dri_map[buf_y_-1][buf_x_]== True and count_distan%2:  
                    #     buf_y_-=1
                    if dx>0 and self.dri_map[buf_y_][buf_x_+1]== True :
                        buf_x_+=1
                    elif dx<0 and self.dri_map[buf_y_][buf_x_-1]== True :  
                        buf_x_-=1
                    elif dy>0 and self.dri_map[buf_y_+1][buf_x_]== True:
                        buf_y_+=1
                    elif dy<0 and self.dri_map[buf_y_-1][buf_x_]== True:  
                        buf_y_-=1
                    else:
                        debug.log('shortest_path_22')        
                    dx = self.id_to_keypoint[current_id][1] - buf_x_
                    dy = self.id_to_keypoint[current_id][0] - buf_y_
                    path_.append(np.array([buf_x_, buf_y_]))
                buf_x = self.id_to_keypoint[list_clost_id_[i]][1]
                buf_y = self.id_to_keypoint[list_clost_id_[i]][0]                
                while len(path_)!=0:
                    path.append(path_.pop())
            current_id = list_clost_id_[i]


        # debug.log('path222  '+str(path))

        # debug.log('start2')        
        buf_x_ = target[0]
        buf_y_ = target[1]       
        dx = buf_x - buf_x_
        dy = buf_y - buf_y_
        path_ = []        
        while((dx!=0 or dy !=0)):
            count_distan+=1             
            if dx>0 and self.dri_map[buf_y_][buf_x_+1]== True:
                buf_x_+=1
            elif dx<0 and self.dri_map[buf_y_][buf_x_-1]== True: 
                buf_x_-=1
            elif dy>0 and self.dri_map[buf_y_+1][buf_x_]== True:
                buf_y_+=1
            elif dy<0 and self.dri_map[buf_y_-1][buf_x_]== True:  
                buf_y_-=1
            else:
                debug.log('shortest_path_33')        
            dx = buf_x - buf_x_
            dy = buf_y - buf_y_            
            path_.append(np.array([buf_x_, buf_y_]))
        while len(path_)!=0:
            path.append(path_.pop())
        # path.append(np.array([buf_x, buf_y]))
        # debug.log('path33333'+str(path))

        keypoint_path_list = []
        keypoint_id_path_list = []        
        for i in range(0, len(list_clost_id_)):
            keypoint_path_list.append(np.array([self.id_to_keypoint[list_clost_id_[i]][1],self.id_to_keypoint[list_clost_id_[i]][0]]))
            keypoint_id_path_list.append(list_clost_id_[i])
        # debug.log('path33333'+str(path))

        return path,keypoint_path_list,keypoint_id_path_list




    def init_(self,ch):
        # debug.log(str('start init tree'))
                # print('搜索失败， 结束！')

        # debug.log(ch[0][0])
        # debug.log(ch[0][1])
        # debug.log(ch[0][2])                
        # for i in range(0, n):
        #     debug.log(ch[i])

        #初始化变量
        for i in range(0, n):
            self.feature_map.append([])
            for j in range(0, n):
                self.feature_map[i].append(0)
        for i in range(0, n):
            self.dri_map.append([])
            for j in range(0, n):
                self.dri_map[i].append(False)
        

        self.square_resolution = int(10)
        self.square_map = []
        for i in range(0, int(n/self.square_resolution)):
            self.square_map.append([])
            for j in range(0, int(n/self.square_resolution)):
                self.square_map[i].append(0)
 
        map_buf = []
        # for i in range(0, n):
        #     map_buf.append(str(ch[i])[2:202])

        for i in range(0, n):
            map_buf.append((ch[i]))
        # for i in range(0, n):
        #     debug.log(ch[i])
        #特征计算

        for i in range(1, n-1):
            for j in range(1, n-1):
                if map_buf[i][j] == '.' or map_buf[i][j] == 'A' or map_buf[i][j] == 'B':
                    self.feature_map[i][j] += 1
                if  map_buf[i-1][j+1] =='#' and map_buf[i-1][j-1] =='#' and map_buf[i-1][j]  =='.' :
                    self.feature_map[i][j] +=2
                if  map_buf[i+1][j+1] =='#' and map_buf[i+1][j-1] =='#' and map_buf[i+1][j] =='.' :
                    self.feature_map[i][j] += 2
                if  map_buf[i-1][j-1] =='#' and map_buf[i+1][j-1] =='#' and map_buf[i][j-1] =='.' :
                    self.feature_map[i][j] += 2
                if  map_buf[i-1][j+1] =='#' and map_buf[i+1][j+1] =='#' and map_buf[i][j+1] =='.' :
                    self.feature_map[i][j] += 2
                if  map_buf[i-1][j+1] =='#' :
                    self.feature_map[i][j] += 1
                if  map_buf[i-1][j-1] =='#' :
                    self.feature_map[i][j] += 1
                if  map_buf[i+1][j+1] =='#' :
                    self.feature_map[i][j] += 1
                if  map_buf[i+1][j-1] =='#' :
                    self.feature_map[i][j] += 1              


        #可行驶区域计算
        for i in range(0, n):
            for j in range(0, n):
                if map_buf[i][j] == 'B' or self.dri_map[i][j] == True:
                    self.dri_map[i][j] == True
                    
                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j-dri_buf <0) :
                            loop = 0 
                        elif map_buf[i][j-dri_buf] == '.' or map_buf[i][j-dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j-dri_buf] = True
                            loop = 1
                        else:
                            loop = 0                        

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j+dri_buf >n-1) :
                            loop = 0               
                        elif map_buf[i][j+dri_buf] == '.' or map_buf[i][j+dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j+dri_buf] = True
                            loop = 1
                        else:
                            loop = 0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i+dri_buf >n-1) :
                            loop = 0 
                        elif map_buf[i+dri_buf][j] == '.' or map_buf[i+dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i+dri_buf][j] = True
                            loop =1
                        else:
                            loop =0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i-dri_buf <0) :
                            loop = 0         
                        elif map_buf[i-dri_buf][j] == '.' or map_buf[i-dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i-dri_buf][j] = True   
                            loop =1
                        else:      
                            loop =0                 


                            
        for i_ in range(0, n-1):
            for j_ in range(0, n-1):
                # i = n-1 - i_
                # j = n-1 - j_
                # i = n-1 - i_
                i= i_
                j = n-1 - j_                
                if map_buf[i][j] == 'B' or self.dri_map[i][j] == True:
                    self.dri_map[i][j] == True
                    
                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j-dri_buf <0) :
                            loop = 0 
                        elif map_buf[i][j-dri_buf] == '.' or map_buf[i][j-dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j-dri_buf] = True
                            loop = 1
                        else:
                            loop = 0                        

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j+dri_buf >n-1) :
                            loop = 0               
                        elif map_buf[i][j+dri_buf] == '.' or map_buf[i][j+dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j+dri_buf] = True
                            loop = 1
                        else:
                            loop = 0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i+dri_buf >n-1) :
                            loop = 0 
                        elif map_buf[i+dri_buf][j] == '.' or map_buf[i+dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i+dri_buf][j] = True
                            loop =1
                        else:
                            loop =0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i-dri_buf <0) :
                            loop = 0         
                        elif map_buf[i-dri_buf][j] == '.' or map_buf[i-dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i-dri_buf][j] = True   
                            loop =1
                        else:      
                            loop =0                 
        for i_ in range(0, n-1):
            for j_ in range(0, n-1):
                i = n-1 - i_
                # i= i_
                j = n-1 - j_
                
                if map_buf[i][j] == 'B' or self.dri_map[i][j] == True:
                    self.dri_map[i][j] == True
                    
                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j-dri_buf <0) :
                            loop = 0 
                        elif map_buf[i][j-dri_buf] == '.' or map_buf[i][j-dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j-dri_buf] = True
                            loop = 1
                        else:
                            loop = 0                        

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j+dri_buf >n-1) :
                            loop = 0               
                        elif map_buf[i][j+dri_buf] == '.' or map_buf[i][j+dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j+dri_buf] = True
                            loop = 1
                        else:
                            loop = 0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i+dri_buf >n-1) :
                            loop = 0 
                        elif map_buf[i+dri_buf][j] == '.' or map_buf[i+dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i+dri_buf][j] = True
                            loop =1
                        else:
                            loop =0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i-dri_buf <0) :
                            loop = 0         
                        elif map_buf[i-dri_buf][j] == '.' or map_buf[i-dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i-dri_buf][j] = True   
                            loop =1
                        else:      
                            loop =0                   
        for i_ in range(0, n-1):
            for j_ in range(0, n-1):
                i = n-1 - i_
                # i= i_
                # j = n-1 - j_
                j = j_
                
                if map_buf[i][j] == 'B' or self.dri_map[i][j] == True:
                    self.dri_map[i][j] == True
                    
                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j-dri_buf <0) :
                            loop = 0 
                        elif map_buf[i][j-dri_buf] == '.' or map_buf[i][j-dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j-dri_buf] = True
                            loop = 1
                        else:
                            loop = 0                        

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(j+dri_buf >n-1) :
                            loop = 0               
                        elif map_buf[i][j+dri_buf] == '.' or map_buf[i][j+dri_buf] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i][j+dri_buf] = True
                            loop = 1
                        else:
                            loop = 0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i+dri_buf >n-1) :
                            loop = 0 
                        elif map_buf[i+dri_buf][j] == '.' or map_buf[i+dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i+dri_buf][j] = True
                            loop =1
                        else:
                            loop =0

                    dri_buf = 0
                    loop = 1
                    while loop:
                        dri_buf += 1
                        if(i-dri_buf <0) :
                            loop = 0         
                        elif map_buf[i-dri_buf][j] == '.' or map_buf[i-dri_buf][j] == 'A' or map_buf[i][j-dri_buf] == 'B':
                            self.dri_map[i-dri_buf][j] = True   
                            loop =1
                        else:      
                            loop =0                 


        for i in range(0, int(n/self.square_resolution)-1):
            for j in range(0, int(n/self.square_resolution)-1):
                if self.dri_map[i*self.square_resolution][j*self.square_resolution] ==True:    
                    self.feature_map[i*self.square_resolution][j*self.square_resolution] = 66
                # if (self.dri_map[i*self.square_resolution][j*self.square_resolution+1] ==False and self.dri_map[i*self.square_resolution][j*self.square_resolution-1] ==False) or (self.dri_map[i*self.square_resolution+1][j*self.square_resolution] ==False and self.dri_map[i*self.square_resolution-1][j*self.square_resolution] ==False):    
                #     self.feature_map[i*self.square_resolution][j*self.square_resolution] = 1
                # if self.dri_map[i*self.square_resolution+1][j*self.square_resolution] ==False and self.dri_map[i*self.square_resolution-1][j*self.square_resolution] ==False:    
                #     self.feature_map[i*self.square_resolution][j*self.square_resolution] = 1                      
        # debug.log('xxxx')
        #确定关键点

        #清空不可行驶区域的特征
        for i in range(0, n):
            for j in range(0, n):
                if self.dri_map[i][j] == False:
                    self.feature_map[i][j] = 0

        for i in range(1, n-1):
            for j in range(1, n-1):
                feature_buf = 0
                if self.feature_map[i][j]>1:
                    if self.feature_map[i][j+1]==1:
                        feature_buf += 1
                    if self.feature_map[i][j-1]==1:
                        feature_buf += 1                    
                    if self.feature_map[i+1][j]==1:
                        feature_buf += 1            
                    if self.feature_map[i-1][j]==1:
                        feature_buf += 1            
                        
                    if feature_buf ==2 or (self.feature_map[i-1][j]==2 and self.feature_map[i+1][j]==2) or (self.feature_map[i][j-1]==2 and self.feature_map[i][j+1]==2):
                        self.feature_map[i][j] = 66
                if(self.feature_map[i][j]==1 and self.feature_map[i][j+1]==0 and self.feature_map[i+1][j+1]>1 and self.feature_map[i-1][j+1]>1):                        
                    self.feature_map[i][j] = 66                # if(self.feature_map[i][j]==1 and self.feature_map[i+1][j]==2 and self.feature_map[i-1][j]==2):                       

                # if self.feature_map[i][j]>0 and self.feature_map[i][j+1]==0 and self.feature_map[i+1][j]==0 and self.feature_map[i][j-1]>0 and self.feature_map[i-1][j]>0 and self.feature_map[i-1][j-1]==0:  
                #     self.feature_map[i][j] = 66                                
                # if self.feature_map[i][j]>0 and self.feature_map[i][j-1]==0 and self.feature_map[i-1][j]==0 and self.feature_map[i][j+1]>0 and self.feature_map[i+1][j]>0 and self.feature_map[i+1][j+1]==0:                        
                #     self.feature_map[i][j] = 66                       
                # if self.feature_map[i][j]>0 and self.feature_map[i][j-1]==0 and self.feature_map[i+1][j]==0 and self.feature_map[i][j+1]>0 and self.feature_map[i-1][j]>0 and self.feature_map[i-1][j+1]==0:              
                #     self.feature_map[i][j] = 66                                
                # if self.feature_map[i][j]>0 and self.feature_map[i][j+1]==0 and self.feature_map[i-1][j]==0 and self.feature_map[i][j-1]>0 and self.feature_map[i+1][j]>0 and self.feature_map[i+1][j-1]==0:                        
                #     self.feature_map[i][j] = 66              
                elif self.feature_map[i][j]>0 and self.feature_map[i][j+1]==0 and self.feature_map[i+1][j]==0 and self.feature_map[i][j-1]>0 and self.feature_map[i-1][j]>0 and self.feature_map[i+1][j+1]==0:  
                    self.feature_map[i][j] = 66                                
                elif self.feature_map[i][j]>0 and self.feature_map[i][j-1]==0 and self.feature_map[i-1][j]==0 and self.feature_map[i][j+1]>0 and self.feature_map[i+1][j]>0 and self.feature_map[i-1][j-1]==0:                        
                    self.feature_map[i][j] = 66                       
                elif self.feature_map[i][j]>0 and self.feature_map[i][j-1]==0 and self.feature_map[i+1][j]==0 and self.feature_map[i][j+1]>0 and self.feature_map[i-1][j]>0 and self.feature_map[i+1][j-1]==0:              
                    self.feature_map[i][j] = 66                                
                elif self.feature_map[i][j]>0 and self.feature_map[i][j+1]==0 and self.feature_map[i-1][j]==0 and self.feature_map[i][j-1]>0 and self.feature_map[i+1][j]>0 and self.feature_map[i-1][j+1]==0:                        
                    self.feature_map[i][j] = 66        
                # if (self.dri_map[i][j+1] ==False and self.dri_map[i][j-1] ==False) or (self.dri_map[i+1][j] ==False and self.dri_map[i-1][j] ==False):    
                #     self.feature_map[i*self.square_resolution][j*self.square_resolution] = 1
                                                                                     
                #     self.feature_map[i][j] = 66
                    # debug.log('ii'+str(i)+'jj'+str(j))
        # debug.log('yyy')
        for i in range(2, n-1):
            for j in range(1, n-2):
                if j>3:
                    if self.feature_map[i][j] == 66 and self.feature_map[i][j-1] == 66:
                        self.feature_map[i][j-1] = 1
                    if self.feature_map[i][j] == 66 and self.feature_map[i-1][j] == 66 and self.feature_map[i-2][j] != 0:
                        self.feature_map[i-1][j] = 1                        
                    if self.feature_map[i][j] == 66 and self.feature_map[i-1][j-1] == 66 and self.feature_map[i-2][j-2] != 0:       #map1
                        self.feature_map[i-1][j-1] = 1             
                    if self.feature_map[i][j] == 66 and self.feature_map[i-1][j+1] == 66:
                        self.feature_map[i-1][j+1] = 1             
                    if self.feature_map[i][j] == 66 and self.feature_map[i-1][j-2] == 66:
                        self.feature_map[i-1][j-2] = 1             
                    if self.feature_map[i][j] == 66 and self.feature_map[i-1][j+2] == 66 and  self.feature_map[i-1][j+1] !=0:
                        self.feature_map[i-1][j+2] = 1             
                    if self.feature_map[i][j] == 66 and self.feature_map[i-2][j-1] == 66:
                        self.feature_map[i-2][j-1] = 1             
                    if self.feature_map[i][j] == 66 and self.feature_map[i-2][j+1] == 66:
                        self.feature_map[i-2][j+1] = 1             
                if j>7:
                    if self.feature_map[i][j] == 66 and self.feature_map[i][j-2] == 66 and self.feature_map[i][j-4] == 66:
                        self.feature_map[i][j-2] = 1;                    
                    if self.feature_map[i][j] == 66 and self.feature_map[i][j-3] == 66 and self.feature_map[i][j-6] == 66:
                        self.feature_map[i][j-3] = 1;                                                      

                    if self.feature_map[i][j-4] == 66 and self.feature_map[i][j-2] == 66 and self.feature_map[i][j-1] != 0:
                        self.feature_map[i][j-2] = 1;                         

                    if self.feature_map[i][j] == 66 and self.feature_map[i-2][j] == 66 and  self.feature_map[i-1][j] != 0 and self.feature_map[i-3][j] != 0:  #map1
                        self.feature_map[i-2][j] = 1;             
                    # if self.feature_map[i-1][j]==2 and self.feature_map[i+1][j]==2:
                    #     self.feature_map[i][j] = 66
                    # if self.feature_map[i][j-1]==2 and self.feature_map[i][j+1]==2:
                    #     self.feature_map[i][j] = 66
                    
        # debug.log('zzz')
        # square_map = []
        # for i in range(0, n):
        #     self.square_map.append([])
        #     for j in range(0, n):
        #         self.square_map[i].append(0)

        # for i in range(0, n):
        #     for j in range(0, n):
        #         if(self.feature_map[i][j] ==66 ):            
        #             self.square_map[int(i/self.square_resolution)][int(j/self.square_resolution)] += 1                
        # for i in range(0, int(n/self.square_resolution)-1):
        #     for j in range(0, int(n/self.square_resolution)-1):
        #         if self.square_map[i][j] < 3 and self.dri_map[i*self.square_resolution][j*self.square_resolution] ==True:    
        #             self.feature_map[i*self.square_resolution][j*self.square_resolution] = 66
                                


        # for i in range(0, int(n/self.square_resolution)-1):
        #     for j in range(0, int(n/self.square_resolution)-1):
        #         if self.dri_map[i*self.square_resolution][j*self.square_resolution] ==True:    
        #             self.feature_map[i*self.square_resolution][j*self.square_resolution] = 66
                                


        #id_to_keypoint 可通过id获取关键点的位置
        for i in range(0, n):
            for j in range(0, n):
                if(self.feature_map[i][j] ==66 ):
                    self.id_to_keypoint.append([i,j])

        #插入树
        for i in range(0, len(self.id_to_keypoint)):
            self.Insert(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0], i)

        # debug.log('self.Search(89,55, 20,0)')
        # start_cost_distance, start_cost_id = self.Search(89,55, 80,0)
        # debug.log('node locs: ', self.id_to_keypoint[start_cost_id])
        # debug.log('node locs: ', self.id_to_keypoint[start_cost_id])
        # for i in range(0, 200):
        #     debug.log(str('star0')+str(self.dri_map[i]))
        #获取关键点的四个方向的最近点
        for i in range(0, len(self.id_to_keypoint)):
            buf1 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,1)
            buf2 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,2)
            buf3 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,3)        
            buf4 = self.Search(self.id_to_keypoint[i][1], self.id_to_keypoint[i][0],50,4)
            self.id_to_nearkeypoinidt.append([])
            self.id_to_nearkeypoinidt[i].append([buf1,buf2,buf3,buf4])
        # debug.log('xxx')

        self.graph_ = {}
        for i in range(0, len(self.id_to_nearkeypoinidt)):
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][0][0]))
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][0][1]))            
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][1][0]))
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][1][1]))
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][2][0]))
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][2][1]))
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][3][0]))
            # debug.log(str(self.id_to_nearkeypoinidt[i][0][3][1]))
            self.graph_[i]={self.id_to_nearkeypoinidt[i][0][0][1]:self.id_to_nearkeypoinidt[i][0][0][0],self.id_to_nearkeypoinidt[i][0][1][1]:self.id_to_nearkeypoinidt[i][0][1][0],self.id_to_nearkeypoinidt[i][0][2][1]:self.id_to_nearkeypoinidt[i][0][2][0],self.id_to_nearkeypoinidt[i][0][3][1]:self.id_to_nearkeypoinidt[i][0][3][0]}
            # graph_[str(i)]={str(self.id_to_nearkeypoinidt[i][0][0][1]):self.id_to_nearkeypoinidt[i][0][0][0],str(self.id_to_nearkeypoinidt[i][0][1][1]):self.id_to_nearkeypoinidt[i][0][1][0],str(self.id_to_nearkeypoinidt[i][0][2][1]):self.id_to_nearkeypoinidt[i][0][2][0],str(self.id_to_nearkeypoinidt[i][0][3][1]):self.id_to_nearkeypoinidt[i][0][3][0]}
        # debug.log(str(self.graph_[500]))
        # debug.log('yyy')
        for i in range(0, len(self.id_to_nearkeypoinidt)):
            if(self.id_to_nearkeypoinidt[i][0][0][1] != 9999):
                self.graph_[self.id_to_nearkeypoinidt[i][0][0][1]].update({i:self.id_to_nearkeypoinidt[i][0][0][0]})
            if(self.id_to_nearkeypoinidt[i][0][1][1] != 9999):                
                self.graph_[self.id_to_nearkeypoinidt[i][0][1][1]].update({i:self.id_to_nearkeypoinidt[i][0][1][0]})
            if(self.id_to_nearkeypoinidt[i][0][2][1] != 9999):                
                self.graph_[self.id_to_nearkeypoinidt[i][0][2][1]].update({i:self.id_to_nearkeypoinidt[i][0][2][0]})
            if(self.id_to_nearkeypoinidt[i][0][3][1] != 9999):                
                self.graph_[self.id_to_nearkeypoinidt[i][0][3][1]].update({i:self.id_to_nearkeypoinidt[i][0][3][0]})
        # debug.log('zzz')
        self.init_Dijkstra(self.graph_)
        # debug.log('xxsssx')
        # current_time = datetime.datetime.now()
        # buf1,buf2 = self.shortest_path(0,400)
        # debug.log('id0 to id400'+str(buf1)+'  distance:'+str(buf2)+'cost time:'+str(datetime.datetime.now()-current_time))
        # for i in range(0, len(buf1)):
        #     debug.log('id:'+str(buf1[i])+' x:'+str(self.id_to_keypoint[buf1[i]][0])+' y:'+str(self.id_to_keypoint[buf1[i]][1]))

        # current_time = datetime.datetime.now()
        # buf1,buf2 = self.shortest_path(0,500)        
        # debug.log('id0 to id500'+str(buf1)+'  distance:'+str(buf2)+'cost time:'+str(datetime.datetime.now()-current_time))
        # for i in range(0, len(buf1)):
        #     debug.log('id:'+str(buf1[i])+' x:'+str(self.id_to_keypoint[buf1[i]][0])+' y:'+str(self.id_to_keypoint[buf1[i]][1]))

        # current_time = datetime.datetime.now()
        # buf1,buf2 = self.shortest_path(0,520)        
        # debug.log('id0 to id520'+str(buf1)+'  distance:'+str(buf2)+str(datetime.datetime.now()-current_time))
        # for i in range(0, len(buf1)):
        #     debug.log('id:'+str(buf1[i])+' x:'+str(self.id_to_keypoint[buf1[i]][0])+' y:'+'cost time:'+str(self.id_to_keypoint[buf1[i]][1]))

# berth_goto_keypoint_dis = []        #没写
# berth_goto_keypoint_cost = []#没写
# keypoint_goto_berth = []#没写

        

        # # debug.log('xxxsss')
        # #调试地图输出
        ########################################
        feature_map_print = []
        for i in range(0, n):
            feature_line_ = ''
            for j in range(0, n):
                if(self.feature_map[i][j] == 0 ):
                    feature_line_ += ' '
                # elif(self.feature_map[i][j] == 1 ):
                #     feature_line_ += 'a'                    
                # elif(self.feature_map[i][j] == 2 ):
                #     feature_line_ += 'b'                      
                # elif(self.feature_map[i][j] == 3 ):
                    # feature_line_ += 'c'                                          
                # elif(self.feature_map[i][j] <= 9 ):
                    # feature_line_ += str(self.feature_map[i][j])
                elif(self.feature_map[i][j] <= 20 ):
                    feature_line_ += '.'                    
                elif(self.feature_map[i][j] >=50 ):
                    feature_line_ += 'W'    

            feature_map_print.append(feature_line_)
        for i in range(0, n):
            debug.log(feature_map_print[i])
        ########################################
        # debug.log('init finsh  tree')

    def Insert(self,x,y,id):
        self.Add(x,y,id,self.root)
        
    def Add(self,x,y,id,node):
        if node==None:
            self.root=QNode(x,y,id)
            return
        
        elif (x>=node.x and y<node.y):
            if node.DL==None:
                node.DL=QNode(x,y,id)
                return
            self.Add(x,y,id,node.DL)
            
        elif (x>node.x and y>=node.y):
            if node.DR==None:
                node.DR=QNode(x,y,id)
                return
            self.Add(x,y,id,node.DR)
            
        elif (x<=node.x and y>node.y):
            if node.UR==None:
                node.UR=QNode(x,y,id)
                return
            self.Add(x,y,id,node.UR)
            
        elif (x<node.x and y<=node.y):
            if node.UL==None:
                node.UL=QNode(x,y,id)
                return
            self.Add(x,y,id,node.UL)

    def QTreePost(self, node):
        if node == None:
            return
        self.QTreePost(node.DL)
        self.QTreePost(node.DR)
        self.QTreePost(node.UR)
        self.QTreePost(node.UL)
        print('(%.d,%.d)--%s'%(node.x,node.y,node.id))        
    def PrintTree(self):
        self.QTreePost(self.root)


    #ori 0 全方向 1 DR 2UR 3UL 4DL
    #ori 0 全方向 1 右下 2右上 3左上 4左下    
    def Search2(self,x0,y0,radius,node,ori):    
        if node == None:
            return [9999,9999]
        buf = self.Search2(x0,y0,radius,node.DL,ori)
        # debug.log('Search'+str(buf))
        if buf[0] <self.near_point_dis:
            self.near_point_dis = buf[0]
            self.near_point_id = buf[1]
        buf = self.Search2(x0,y0,radius,node.DR,ori)
        if buf[0] <self.near_point_dis:
            self.near_point_dis = buf[0]
            self.near_point_id = buf[1]
        buf = self.Search2(x0,y0,radius,node.UR,ori)
        if buf[0] <self.near_point_dis:
            self.near_point_dis = buf[0]
            self.near_point_id = buf[1]
        buf = self.Search2(x0,y0,radius,node.UL,ori)
        if buf[0] <self.near_point_dis:
            self.near_point_dis = buf[0]
            self.near_point_id = buf[1]
        # debug.log('c')
        if ori ==1:
            if (node.x>=x0  and node.y>y0)==0:
                return [self.near_point_dis,self.near_point_id]
        if ori ==2:
            if  (node.x<x0  and node.y>=y0)==0:
                return [self.near_point_dis,self.near_point_id]
        if ori ==3:
            if (node.x<=x0 and node.y<y0)==0:
                return [self.near_point_dis,self.near_point_id]
        if ori ==4:
            if (node.x>x0  and node.y<=y0)==0:
                return [self.near_point_dis,self.near_point_id]
          
        dis_ = 0
        if node.x-x0>0:
            dis_ += (node.x-x0)
        else:
            dis_ += (x0 - node.x)
        if node.y-y0>0:
            dis_ += (node.y-y0)
        else:
            dis_ += (y0 - node.y)
        if(dis_>self.near_point_dis):
            # debug.log('dis'+str(dis_)+'self.near_point_dis'+str(self.near_point_dis))
            # debug.log(str(node.x)+'node'+str(node.y)+'x0'+str(x0)+'y0'+str(y0)+'ori'+str(ori))
            return [self.near_point_dis,self.near_point_id]
        buf_x =  x0
        buf_y =  y0        
        dx = node.x - buf_x
        dy = node.y - buf_y
        count_distan = 0

        while((dx!=0 or dy !=0) and count_distan < self.near_point_dis ):
            dx = node.x - buf_x
            dy = node.y - buf_y
            count_distan+=1
            # if dx>0 and self.dri_map[buf_y][buf_x+1]== True and count_distan%2==0:
            #     buf_x+=1
            # elif dx<0 and self.dri_map[buf_y][buf_x-1]== True and count_distan%2==0: 
            #     buf_x-=1
            # elif dy>0 and self.dri_map[buf_y+1][buf_x]== True and count_distan%2:
            #     buf_y+=1
            # elif dy<0 and self.dri_map[buf_y-1][buf_x]== True and count_distan%2:  
            #     buf_y-=1
            if dx>0 and self.dri_map[buf_y][buf_x+1]== True :
                buf_x+=1
            elif dx<0 and self.dri_map[buf_y][buf_x-1]== True : 
                buf_x-=1
            elif dy>0 and self.dri_map[buf_y+1][buf_x]== True :
                buf_y+=1
            elif dy<0 and self.dri_map[buf_y-1][buf_x]== True :  
                buf_y-=1                
            elif dy ==0 and dx ==0:
                dy=0
            # elif dx!=0 and count_distan%2==0:
                # count_distan = self.near_point_dis
            # elif dy!=0 and count_distan%2:
            else:
                count_distan = self.near_point_dis                

        if((dx==0 and dy ==0)and count_distan < self.near_point_dis):
            # debug.log(str(node.x)+'node'+str(node.y)+'x0'+str(x0)+'y0'+str(y0)+'ori'+str(ori))
            # debug.log(str(node.id)+'count_distan'+str(count_distan)+'wz'+str(self.id_to_keypoint[node.id]))
            self.near_point_dis = count_distan
            self.near_point_id = node.id
            return [int(count_distan), int(node.id)]
        else:
            return [self.near_point_dis,self.near_point_id]



    # @brief: 搜索(xo,yo)附近最近的关键点
    # @param radius: 搜索radius距离代价内的关键点
    # @param ori:   0 全方向 1 DR 2UR 3UL 4DL
    #               0 全方向 1 右下 2右上 3左上 4左下 
    # @return:  成功 - [最近关键点的代价距离，最短关键点的id]
    #           失败 - [9999,9999]
    def Search(self,x0,y0,radius,ori):
        self.near_point_dis = radius
        self.near_point.clear()
        self.near_point_dis = 9999
        self.near_point_id = 9999
        return self.Search2(x0,y0,radius,self.root,ori)


###########################################################################
###########################################################################
###########################################################################

















mytree = QuadTree()








def Init():
    for i in range(0, n):
        line = input()
        ch.append([c for c in line.split(sep=" ")])
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    boat_capacity = int(input())
    okk = input()


    mytree.init_(ch)


    print("OK")
    sys.stdout.flush()


def Input():
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        gds[x][y] = val
    for i in range(robot_num):
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    return id

if __name__ == "__main__":
    debug.log_title()   # 打印debug分隔符
    debug.log("debug!") # 打印debug信息
    Init()

    for zhen in range(1, 15001):
        id = Input()
        for i in range(robot_num):
            print("move", i, random.randint(0, 3))
            sys.stdout.flush()
        print("OK")
        sys.stdout.flush()
