import sys
import os
sys.path.append(os.getcwd())
from driver import console # 交互模块
from policy import Policy
import debug    # debug库
import time
#### 调试用，提交时请注释掉！ ####
# import ui


if __name__ == "__main__":
    # 打印debug分隔符
    debug.log_title()   
    
    # ui显示（可能是多线程的问题，UI_init时会导致log乱码）
    if 'ui' in sys.modules:
        console.add_task(ui.UI_init, init=True)
        console.add_task(ui.UI_update)

    # 决策器
    ply = Policy(console)
    console.add_task(task=ply.tasks_init, init=True)
    console.add_task(task=ply.assign_tasks)

    # 初始化
    console.initialize() 
    debug.log("Init finished!")
    
    # 执行任务
    for _ in range(1, 15001):      
        console.frame_update()
        
        if console.frame == 15000:  # 结束
            break
    # time.sleep(100000)
    # 任务结束
    debug.log('Task over!')
       
    # ui关闭
    if 'ui' in sys.modules:
        ui.UI_close()
