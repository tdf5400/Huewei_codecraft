"""
自己用多线程显示游戏界面

使用方法：
console.add_task(ui.UI_init, init=True)
console.add_task(ui.UI_update)

TODO:
    1. 货物自动过期，发增量即可减少主函数运算量
    
Attention: 可能是多线程的问题，UI_init时会导致log乱码
"""


F_SWITCH_UI = True # 开关-是否显示UI
LOOP_MS = 50 # 信息输出间隔(ms)



import numpy as np
from driver import Map, Driver, map_class, Robot, Cargo, Berth
import sys
import math


if F_SWITCH_UI:
    from multiprocessing import Process, Queue
    import time
    from PyQt5.QtCore import *
    from PyQt5.QtGui import *
    from PyQt5.QtWidgets import *

    UI_task = None
    UI_queue = Queue()
    
    def UI_init(ctl:Driver):
        """ UI初始化 """
        global UI_task, UI_queue
        berth_locs = [i.raw_loc for i in ctl.berths]
        UI_task = Process(target=UI_process, args=(ctl.map, berth_locs, UI_queue, LOOP_MS), daemon=True) 
        UI_task.start() # 开始任务

    time_update = 0 # 下次更新信息的时间戳
    def UI_update(ctl:Driver):
        """ UI更新任务 """
        global UI_queue, UI_task, time_update
        if UI_task.is_alive():  # 任务存活
            now_time = time.time()
            
            # 间隔过短则跳过
            if now_time < time_update:  
                return
            
            time_update = now_time + (LOOP_MS / 1000)
            
            # 机器人坐标
            bots = [bot.loc for bot in ctl.bots]
            
            # 货物坐标+价值
            cargos = []
            for cargo in ctl.cargos.queue:
                # 已拿取，则跳过
                if cargo.is_taken():
                    continue
                cargos.append([cargo.loc, cargo.value])
                
            # 泊口信息
            berths = [[str(i.value()) + '/' + str(i.count())] for i in ctl.berths]
            
            # 船信息(取不在移动中的)
            boats = []
            for i in ctl.boats:
                if i.is_arrive():
                    boats.append(i.target)
                else:                    
                    boats.append(-2)
                
            # 传入消息  
            buf = [bots, cargos, berths, boats]
            UI_queue.put(buf)          

    def UI_process(map:Map, berth_locs:list, msg_queue:Queue, loop_ms=500):
        """ 多进程函数，在另一个进程跑绘图 """
        app = QApplication(sys.argv)
        win = UI(map, berth_locs, msg_queue=msg_queue, loop_ms=loop_ms)
        win.show()              # 窗口显示
        sys.exit(app.exec_())
        
    def UI_close():
        """ 关闭窗口 """
        global UI_task
        UI_task.terminate()

    class UI(QWidget):
        win_size = 1000 # 窗口边长分辨率px(1:1)
        block_size = 0  # 1x1格子的绘制像素尺寸
        bolck_sign = {
            map_class['B']: [102,102,131],  # 0泊位，灰
            map_class['.']: [51, 51, 51],   # 1空地，浅黑
            map_class['*']: [36,144,241],   # 2海洋，蓝
            map_class['#']: [30, 30, 30],   # 3障碍，深黑
        }
        objs_sign = {
            'robot': ['#', QColor(22, 212, 107)],   # 机器人
            'cargo': ['$', QColor(206, 145, 120)],  # 货物
            'berth': ['B', QColor(255, 255, 255)],  # 泊口
            'boat': ['@', QColor(255, 18, 57)],    # 船
        }

        draw_buf = [] # 绘制文本的信息，用于timer传递绘制信息 [[char, color], loc]

        def __init__(self, map:Map, berth_locs:list, msg_queue:Queue, loop_ms=500) -> None: 
            """
            初始化
            map - 地图数组
            msg_queue - 多线程的消息列表，每帧格式为：[bots, cargos, ]
            """
            super().__init__()
            self.msg_queue = msg_queue
            self.berth_locs = berth_locs

            # 绘制背景图
            h, w = map.shape
            block_size = math.ceil(self.win_size / max(h, w))
            self.block_size = block_size
            background = np.zeros(((h+3)*block_size, (w+2)*block_size, 3), dtype=np.uint8)
            for y, line in enumerate(map):
                for x, sign in enumerate(line):
                    x1 = x*block_size
                    x2 = x1 + block_size
                    y1 = y*block_size
                    y2 = y1 + block_size
                    background[y1:y2, x1:x2] = self.bolck_sign[sign]           
            h, w, d = background.shape
            QImg = QImage(background.data, w, h, w*d, QImage.Format_RGB888)
            self.img_bg = QPixmap.fromImage(QImg)
        
            # 设置背景 
            self.resize(w, h)   # 修改窗口为地图大小
            palette = QPalette()
            palette.setBrush(QPalette.Background, QBrush(self.img_bg))
            self.setPalette(palette)
            # self.show()

            # 定时任务
            self.timer = QTimer(self) # 计时器，定时触发某函数
            self.timer.timeout.connect(self.timer_task) # 计时器执行的任务
            self.timer.start(loop_ms)

        def paintEvent(self, event):
            painter = QPainter(self)   # 绘制器

            for [char, color], loc, scale in self.draw_buf: 
                str_len = len(char) # 字符长度
                loc = loc * self.block_size
                # 设置画笔颜色
                painter.setPen(color)
                # 设置字体
                painter.setFont(QFont("SimSun", int(self.block_size*scale)))
                # 指定绘图区域，对齐方式和绘制内容
                painter.drawText(QRect(int(loc[0]), int(loc[1]), int(self.block_size * scale * str_len),
                                       int(self.block_size * scale * 1.5)), Qt.AlignCenter, char)
        def timer_task(self):
            if self.msg_queue.empty():  # 没有信息在队列里
                return
            while not self.msg_queue.empty(): # 取队列最新帧
                buf = self.msg_queue.get()
            # buf = [[Robot(0, [0, 0], 0, 1), Robot(0, [5, 0], 0, 1)], [Cargo([6, 6], 100, 0), Cargo([8, 6], 80, 0)]] # [bots, cargos, ]

            bots = buf[0] 
            cargos = buf[1]
            berths = buf[2]
            boats = buf[3]

            self.draw_buf = [] # 清空绘制内容
            # 泊口
            for id, loc in enumerate(self.berth_locs):
                brash = [str(id), self.objs_sign['berth'][1]]
                self.draw_buf.append([brash, loc, 3]) # id
                
            for id, info in enumerate(berths):  # 信息画在下方              
                loc = self.berth_locs[id] + [-2, 4]    # 坐标偏移
                brash = [info[0], self.objs_sign['berth'][1]]
                self.draw_buf.append([brash, loc, 2]) # 当前价值 / 载货量
            
            # 机器人
            for id, loc in enumerate(bots):
                brash = [str(id), self.objs_sign['robot'][1]]
                self.draw_buf.append([brash, loc, 2.5]) # id

            # 货物
            for loc, val in cargos:
                brash = self.objs_sign['cargo']
                self.draw_buf.append([brash, loc, 1.8]) # 货物
                self.draw_buf.append([[str(val), brash[1]], loc+np.array([0, 2.5]), 1.7]) # 价值
                
            # 船
            for boat_id, berth_id in enumerate(boats):
                if berth_id == -2:  # 无效值
                    continue
                elif berth_id == -1:  # 不在泊口
                    loc = np.array([boat_id * 2, 0])
                else:   # 在泊口
                    loc = self.berth_locs[berth_id] + [2, -1]
                    
                brash = [str(boat_id), self.objs_sign['boat'][1]]
                self.draw_buf.append([brash, loc, 2.5])
                

            self.repaint()  # 提交重新绘制任务

      


if __name__ == "__main__":
    mymap = np.array([
        [1, 1, -1, 1,  1, 1, 1, 1, 1, 1, 1],
        [1, 1, -1, 1,  1, -1, 1, 1, 1, 1, 1],
        [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
        [1, 1,  1, 1, -1, 1, 1, 1, 1, 1, 1],
        [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
        [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
        [1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1],
    ])
    map = Map(mymap)

    ctl = Driver()
    ctl.map = map
    UI_init(ctl)
    ctl.cargos.put(Cargo([6, 0], 100, 0))
    ctl.cargos.put(Cargo([2, 3], 80, 0))
    while True:
        ctl.bots[0] = Robot(0, [0, np.random.randint(0, 8)], 0, 1)
        ctl.bots[1] = Robot(0, [5, np.random.randint(0, 8)], 0, 1)
        UI_update(ctl)
    