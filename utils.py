"""
计算工具
"""
import numpy as np
import debug


def path2vec(path):
    """ 路径转方向 """
    last_loc = path[0]
    vecs = []
    for p in path[1:]:
        sub = p - last_loc

        if sub[1] == -1:  # 'w'
            vecs.append('w')
        elif sub[1] == 1:  # 's'
            vecs.append('s')
        elif sub[0] == -1:  # 'a'
            vecs.append('a')
        elif sub[0] == 1:  # 'd'
            vecs.append('d')

        last_loc = p
    return vecs


def step2vec(bot_loc, next_step):
    if next_step is None:
        return
    sub = next_step - bot_loc
    if sub[1] == -1:
        return 'w'
    elif sub[1] == 1:  # 's'
        return 's'
    elif sub[0] == -1:  # 'a'
        return 'a'
    elif sub[0] == 1:  # 'd'
        return 'd'


def calculate_distance(loc1, loc2):
    """计算两点之间的距离"""
    return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])


if __name__ == "__main__":
    """ 测试A* """
