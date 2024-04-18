F_SWITCH = True # 开关-是否输出调试

import numpy as np
import os


"""
日志打印
"""
LEVEL_DEBUG = 1
LEVEL_INFO = 2
LEVEL_WARNING = 3
LEVEL_ERROR = 4
LEVEL_CRITICAL = 5
if F_SWITCH:
    import logging
    log_file = os.path.abspath('./test.log')
    disp_level = logging.DEBUG # 打印日志的级别，不会打印级别低的log
    logging.basicConfig(filename=log_file, filemode="w", format="%(asctime)s %(name)s:%(levelname)s:%(message)s", datefmt="%d-%M-%Y %H:%M:%S", level=disp_level)

def log_title():
    """ 打印抬头，做分隔符 """
    [log(".......................") for i in range(4)]
    log("......New one.......")
    [log(".......................") for i in range(4)]
    
def log(*msg, level=LEVEL_DEBUG) -> None:
    """ 打印日志 """
    if not F_SWITCH: # 不打印
        return 
    elif level == LEVEL_DEBUG:
        logging.debug(msg)
    elif level == LEVEL_INFO:
        logging.info(msg)
    elif level == LEVEL_WARNING:
        logging.warning(msg)
    elif level == LEVEL_ERROR:
        logging.error(msg)
    elif level == LEVEL_CRITICAL:
        logging.critical(msg)
    else:
        raise IOError("Invalid param")

    
if __name__ == "__main__":
    print('Debug flag is: ', F_SWITCH)
    log_title()
    log("debug!")
    print('Test finished!')