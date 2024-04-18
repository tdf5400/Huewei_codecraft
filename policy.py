import debug
from utils import calculate_distance
import numpy as np

total_frame = 15000
cargo_alive_frame = 1000 - 10
berth2berth_frame = 500
boat_load_cargo_frame = 20  # 预留的装货时间


class Policy:
    def __init__(self, driver):
        self.driver = driver
        self.bots = driver.bots
        self.boats = driver.boats
        self.berths = driver.berths
        self.cargos = driver.cargos

    def tasks_init(self, driver):
        self.frame = driver.frame
        self.assign_robots2berths()
        # 迷宫
        if driver.map_id == 1:  
            self.assign_robots2berths(nums_berths=8)
        # 开阔
        elif driver.map_id == 2:  
            self.assign_robots2berths(nums_berths=9)
        # 其他
        else:
            self.assign_robots2berths(nums_berths=10)

    def assign_tasks(self, driver):
        """为机器人和船分配任务"""
        self.frame = driver.frame
        
        # 迷宫
        if driver.map_id == 1:  
            # if self.frame == 9000:
            #     self.assign_robots2berths(nums_berths=7)
            if self.frame == 10000:
                self.assign_robots2berths(nums_berths=6)
            if self.frame == 13500:
                self.assign_robots2berths(nums_berths=3)
            self.assign_robots()
            self.assign_boats()
            self.check_berth_alive()
        # 开阔
        elif driver.map_id == 2:        
            # if self.frame == 10000:
            #     self.assign_robots2berths(nums_berths=7)      
            if self.frame == 12500:
                self.assign_robots2berths(nums_berths=7)
            if self.frame == 13500:
                self.assign_robots2berths(nums_berths=5)
            self.assign_robots()
            self.assign_boats()
            self.check_berth_alive()
        # 其他地图
        else:   
            self.assign_robots()
            self.assign_boats()
            self.check_berth_alive()   
            if self.frame == 12000:
                self.assign_robots2berths(nums_berths=5)
            if self.frame == 13500:
                self.assign_robots2berths(nums_berths=3)

    def assign_robots2berths(self, nums_berths=10):
        for berth in self.berths:
            berth.assigned_robots = []

        selected_berths = self.select_berths(nums_berths)

        """分配机器人到渡口"""
        for bot in self.bots:
            if bot.alive is False:
                continue
            valid_berths = {}
            for berth in selected_berths:
                dist = self.driver.get_berth_distance(berth.id, bot.loc)
                if dist > 0:
                    valid_berths[berth] = {
                        'nums_robots': len(berth.assigned_robots),
                        'distance': dist
                    }

            # 如果有符合条件的渡口，先按已分配机器人数量排序，再按泊口离机器人距离排序
            if valid_berths:
                best_berth = min(valid_berths,
                                 key=lambda x: (valid_berths[x]['nums_robots'], valid_berths[x]['distance']))
                self.berths[best_berth.id].assigned_robots.append(bot.id)
                bot.assigned_berth = best_berth.id
                # debug.log(f"机器人 {bot.id} 被分配到渡口 {bot.assigned_berth}")
            else:
                # 如果没有合适的渡口，设置机器人的alive参数为False，清空路径
                bot.alive = False
                bot.set_path([])
                # debug.log(f"机器人 {bot.id} 未找到合适的渡口，已设置为非活动状态")

    def assign_robots(self):
        """为机器人分配搬运货物的任务"""
        for bot in self.bots:
            if bot.status != 1 or self.cargos.empty() or not bot.alive:
                continue

            if bot.no_path():
                bot_goto_flag = self.determine_bot_road(bot)
                if bot_goto_flag < 0:
                    continue

    def select_berths(self, nums_berths=10):
        """根据给定的参数筛选泊口"""
        alive_berths = [berth for berth in self.berths if berth.alive is True]

        if nums_berths >= len(alive_berths):
            return alive_berths

        # 筛选掉收益过低的泊口，先找一个go_time最小的泊口，然后让泊口之间两两的最小距离最大
        # mean_value = np.mean([berth.sum_values for berth in alive_berths])
        # target_berth = [berth for berth in alive_berths if berth.sum_values * 1.5 >= mean_value]

        selected_berths = [
            min([berth for berth in alive_berths if berth.alive is True],
                key=lambda x: x.go_time)]

        while len(selected_berths) < nums_berths:
            next_berth = max(
                (berth for berth in alive_berths if berth not in selected_berths),
                key=lambda x: min(calculate_distance(x.loc, selected.loc) for selected in selected_berths)
            )
            selected_berths.append(next_berth)

        # 将未被选中的泊口的alive属性设置为False
        for berth in alive_berths:
            if berth not in selected_berths:
                self.set_berth_alive(berth.id, alive=False)
        return selected_berths

    def determine_bot_road(self, bot):
        """决定机器人的行进路线"""
        flag = -1
        if bot.goods == 0:
            flag = self.find_best_cargo(bot)
        elif bot.goods == 1:
            flag = self.bot_goto_berth(bot)
        return flag

    def bot_goto_berth(self, bot):
        berth_loc = min(self.berths[bot.assigned_berth].loc_scores[:4],
                        key=lambda loc: calculate_distance(loc, bot.loc))
        flag = self.driver.bot_goto(bot.id, berth_loc)
        return flag

    def assign_boats(self):
        """为船只分配目的地"""
        # 轮船和泊位之间有到达、装载、驶离三种动作。每一帧在泊位处将依次执行到达、驶离、装载三种动作。
        for boat in self.boats:
            if boat.status != 1:
                continue
            debug.log(
                f'{self.frame}, boat {boat.id}, capacity:{boat.capacity}, use_capacity: {boat.size()}, target: {boat.target}')
            if boat.target == -1 or self.berths[boat.target].count() == 0:
                available_berths = [berth for berth in self.berths if not berth.locked()]
                if len(available_berths) < 1:  # 没有可用泊口
                    continue
                # 如果有可用的泊口，按预计装货量排序，如果相同则优先选择压货少的泊口
                target_berth = max(available_berths, key=lambda x: (min(x.count(), boat.free()), -x.count()))
                if self.frame + target_berth.go_time + berth2berth_frame + boat_load_cargo_frame < total_frame:
                    # debug.log(f"boat {boat.id} used: {boat._used} target: {boat.target} {boat.move(target_berth.id)}")
                    # 提前出发卸货
                    if boat.size() / boat.capacity >= 0.9 and boat.free() < target_berth.count():
                        self.driver.boat_goto(boat, -1)
                        boat.arrive_frame = self.frame + self.berths[boat.target].go_time
                    else:
                        self.driver.boat_goto(boat, target_berth.id)
                        boat.arrive_frame = self.frame + target_berth.go_time if boat.target == -1 else self.frame + berth2berth_frame
                    continue

            # 容量到达上限或者快结束时直接出发
            go_time = self.berths[boat.target].go_time
            if boat.target != -1 and (boat.full() or self.frame + go_time >= total_frame):
                # debug.log(f"boat {boat.id} used: {boat._used} target: {boat.target} {boat.move(-1)}")
                boat.clear()
                self.driver.boat_goto(boat, -1)
                boat.arrive_frame = self.frame + go_time
                continue

    def find_best_cargo(self, bot):
        """找到价值与距离比最高的货物并锁定"""
        potential_cargos = []
        flag = -1

        for idx, cargo in enumerate(self.cargos.queue):
            # 距离由两部分组成，取货物时泊口到货物的距离，以及卸货物时货物到最优泊口的距离，两个泊口可以相同也不可以不相同
            dist = self.driver.get_berth_distance(bot.assigned_berth, cargo.loc)
            if cargo.locked() or cargo.born_frame + cargo_alive_frame < self.frame + dist or dist < 0:
                continue
            target_berth = None
            # target_berth = self.find_best_berth_for_cargo(cargo)

            cargo_alive_time = cargo_alive_frame + cargo.born_frame - self.frame
            # dist += self.driver.get_berth_distance(target_berth.id, cargo.loc)
            value_ratio = cargo.value / dist + cargo.value / cargo_alive_time * 0.2

            potential_cargos.append((value_ratio, idx, cargo, target_berth))

        potential_cargos.sort(reverse=True, key=lambda x: x[0])

        for _, idx, cargo, target_berth in potential_cargos:
            flag = self.driver.bot_goto(bot.id, cargo.loc)
            if flag == 0:
                # bot.assigned_berth = target_berth.id
                self.cargos.queue[idx].lock(bot.id)
                return flag
        return flag

    def find_best_berth_for_cargo(self, cargo):
        # 筛选出能到达且还开放的港口
        available_berths = [berth for berth in self.berths if berth.alive is True and cargo.berths_dist[berth.id] > 0]
        # 筛选掉已经饱和的港口
        not_full_berth = []
        for berth in available_berths:
            if berth.locked():
                boat = self.boats[berth.for_boat()]
                if boat.size() + berth.count() < boat.capacity:
                    not_full_berth.append(berth)
            else:
                min_capacity = min([boat.capacity for boat in self.boats])
                if berth.count() < min_capacity:
                    not_full_berth.append(berth)

        target_berths = not_full_berth if len(not_full_berth) > 0 else available_berths
        target_berth = min(target_berths, key=lambda berth: cargo.berths_dist[berth.id])
        return target_berth

    def check_berth_alive(self):
        for berth in self.berths:
            if berth.alive is False:
                continue
            available_boats = []
            for boat in self.boats:
                if boat.target == -1:
                    if boat.arrive_frame + 2 * berth.go_time + boat_load_cargo_frame < total_frame:
                        available_boats.append(boat)
                elif boat.target == berth.id:
                    if max(boat.arrive_frame, self.frame) + berth.go_time + boat_load_cargo_frame < total_frame:
                        available_boats.append(boat)
                else:
                    if max(boat.arrive_frame,
                           self.frame) + berth2berth_frame + berth.go_time + boat_load_cargo_frame < total_frame:
                        available_boats.append(boat)

            if len(available_boats) == 0:
                self.set_berth_alive(berth.id, alive=False)

    def set_berth_alive(self, berth_id, alive=False):
        self.berths[berth_id].alive = alive
        debug.log(f"close berth {berth_id}")
        self.assign_robots2berths()
        # 当有泊口关闭时，所有携带货物的机器人重新规划去泊口的路线
        for bot in self.bots:
            if bot.alive and bot.goods == 1:
                self.bot_goto_berth(bot)
