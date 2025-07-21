import copy
import random
import time

import numpy as np

from KJH.KJH_vrpb import Route


class SolPool:
    def __init__(self, current_sol, capa, node_demand, node_type, random_cost):
        total_cost = sum(route.cost for route in current_sol)
        self.best_sol = copy.deepcopy(current_sol)
        self.best_cost = total_cost
        self.current_best_sol = copy.deepcopy(current_sol)
        self.current_best_cost = total_cost
        self.current_sol = current_sol
        self.current_cost = total_cost
        self.pool = []
        self.sol_hash = {}
        self.cost_hash = {}
        self.capa = capa
        self.node_type = node_type
        self.node_demand = node_demand
        self.random_cost = random_cost
        for i in range(len(current_sol)):
            route_hash = self.get_hash(current_sol[i].hist)
            self.pool.append(route_hash)
            self.cost_hash[route_hash] = current_sol[i].cost
            self.sol_hash[route_hash] = current_sol[i].hist.copy()

    def reset(self):
        self.current_best_sol = copy.deepcopy(self.best_sol)
        self.current_best_cost = self.best_cost
        self.current_sol = copy.deepcopy(self.best_sol)
        self.current_cost = self.best_cost

    def calculate_current_cost(self):
        self.current_cost = sum(route.cost for route in self.current_sol)

    def get_hash(self, route):
        return sum(self.random_cost[i] for i in route)

    def add_pool(self, route, cost):
        route_hash = self.get_hash(route)
        if route_hash in self.sol_hash:
            if cost < self.cost_hash[route_hash]:
                self.cost_hash[route_hash] = cost
                self.sol_hash[route_hash] = route.copy()
            return
        self.sol_hash[route_hash] = route.copy()
        self.pool.append(route_hash)
        self.cost_hash[route_hash] = cost

    def make_sol(self, route, cost):
        line_load = 0
        back_load = 0
        line_idx = 1  # back이 시작하는 부분
        for i in range(len(route)):
            if self.node_type[route[i]] == 1:
                line_idx = i + 1
                line_load += self.node_demand[route[i]]
            elif self.node_type[route[i]] == 2:
                back_load += self.node_demand[route[i]]
        return Route(route, cost, line_load, back_load, line_idx)


class ILS_RVND:
    def __init__(self, K, dist_mat):
        self.K = K
        self.dist_mat = dist_mat
        self.improved_route = [0] * K
        self.shift_status = np.zeros((K, K), int)  # 루트끼리 비교, 개선없음, 불가 : -1, 방문 안함 : 0, 개선 값 : 0보다 큼
        self.shift_status_pos = {}  # 루트끼리 비교, 개선없음, 불가 : -1, 방문 안함 : 0, 개선 값 : 0보다 큼
        self.swap11_status = np.zeros((K, K), int)
        self.swap11_status_pos = {}
        self.swap21_status = np.zeros((K, K), int)
        self.swap21_status_pos = {}
        self.swap22_status = np.zeros((K, K), int)
        self.swap22_status_pos = {}
        self.status_list = [self.shift_status, self.swap11_status, self.swap21_status, self.swap22_status]

    def reset_status(self):
        for status in self.status_list:
            status[:] = 0

    def status_update(self, i, j):
        self.improved_route[i] = 1
        self.improved_route[j] = 1
        for status in self.status_list:
            status[i, :] = 0
            status[:, i] = 0
            status[j, :] = 0
            status[:, j] = 0

    def shift(self, spool):
        improved = False
        current_sol = spool.current_sol
        len_sol = len(current_sol)

        for i in range(len_sol):
            for j in range(len_sol):
                if i == j or self.shift_status[i, j] != 0:
                    continue

                # 빈 차량 또는 line이 1개이고 back이 여러 개인 경우 shift 안함
                if current_sol[i].line_idx == 1 or (current_sol[i].line_idx == 2 and len(current_sol[i].hist) > 3):
                    continue

                give_route = current_sol[i]
                take_route = current_sol[j]
                i_to_j_best = 0
                i_to_j_pos = 0

                # line to line
                for x in range(1, give_route.line_idx):
                    if spool.node_demand[give_route.hist[x]] + take_route.line_load > spool.capa:
                        continue
                    pre_x_cost = self.dist_mat[give_route.hist[x - 1]][give_route.hist[x]] + \
                                 self.dist_mat[give_route.hist[x]][give_route.hist[x + 1]]
                    after_x_cost = self.dist_mat[give_route.hist[x - 1]][give_route.hist[x + 1]]
                    for y in range(1, take_route.line_idx):
                        # 개선 비용 = 개선 비용 - 추가 비용
                        after_y_cost = self.dist_mat[take_route.hist[y - 1]][give_route.hist[x]] + \
                                       self.dist_mat[give_route.hist[x]][take_route.hist[y]]
                        delta = pre_x_cost - after_x_cost - after_y_cost
                        if i_to_j_best < delta:
                            i_to_j_best = delta
                            i_to_j_pos = (x, y)

                # back to back
                for x in range(give_route.line_idx, len(give_route.hist) - 1):
                    if spool.node_demand[give_route.hist[x]] + take_route.back_load > spool.capa:
                        continue
                    pre_x_cost = self.dist_mat[give_route.hist[x - 1]][give_route.hist[x]] + \
                                 self.dist_mat[give_route.hist[x]][give_route.hist[x + 1]]
                    after_x_cost = self.dist_mat[give_route.hist[x - 1]][give_route.hist[x + 1]]
                    for y in range(take_route.line_idx, len(take_route.hist) - 1):
                        # 개선 비용 = 개선 비용 - 추가 비용
                        after_y_cost = self.dist_mat[take_route.hist[y - 1]][give_route.hist[x]] + \
                                       self.dist_mat[give_route.hist[x]][take_route.hist[y]]
                        delta = pre_x_cost - after_x_cost - after_y_cost
                        if i_to_j_best < delta:
                            i_to_j_best = delta
                            i_to_j_pos = (x, y)

                self.shift_status[i, j] = i_to_j_best if i_to_j_best else -1
                self.shift_status_pos[(i, j)] = i_to_j_pos
        i, j = np.unravel_index(np.argmax(self.shift_status), self.shift_status.shape)
        if self.shift_status[i, j] > 0:
            improved = True
            x, y = self.shift_status_pos[(i, j)]
            # line_idx 업데이트, load 업데이트, hash 검사
            give_hist = current_sol[i].hist
            v = give_hist.pop(x)

            take_hist = current_sol[j].hist
            take_hist.insert(y, v)

            cost1 = sum(
                self.dist_mat[give_hist[i - 1]][give_hist[i]] for i in range(1, len(give_hist)))
            cost2 = sum(
                self.dist_mat[take_hist[i - 1]][take_hist[i]] for i in range(1, len(take_hist)))
            spool.current_sol[i] = spool.make_sol(give_hist, cost1)
            spool.current_sol[j] = spool.make_sol(take_hist, cost2)
            # spool.add_pool(give_hist, cost1)
            # spool.add_pool(take_hist, cost2)

            self.status_update(i, j)

        return improved

    def swap11(self, spool):
        improved = False
        current_sol = spool.current_sol
        len_sol = len(current_sol)
        for i in range(len_sol):
            for j in range(i + 1, len_sol):
                if self.swap11_status[i, j] != 0:
                    continue

                swap1_route = current_sol[i]
                swap2_route = current_sol[j]
                i_to_j_best = 0
                i_to_j_pos = 0
                if swap1_route.line_idx > 1 and swap2_route.line_idx > 1:
                    # line to line
                    for x in range(1, swap1_route.line_idx):
                        pre_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap1_route.hist[x]] + \
                                     self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]]
                        for y in range(1, swap2_route.line_idx):
                            if swap2_route.line_load + spool.node_demand[swap1_route.hist[x]] - spool.node_demand[
                                swap2_route.hist[y]] > spool.capa:
                                continue
                            if swap1_route.line_load + spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                                swap1_route.hist[x]] > spool.capa:
                                continue
                            pre_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap2_route.hist[y]] + \
                                         self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]]

                            after_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap2_route.hist[y]] + \
                                           self.dist_mat[swap2_route.hist[y]][swap1_route.hist[x + 1]]

                            after_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap1_route.hist[x]] + \
                                           self.dist_mat[swap1_route.hist[x]][swap2_route.hist[y + 1]]

                            # 개선 비용 = 개선 비용 - 추가 비용
                            delta = pre_x_cost + pre_y_cost - after_x_cost - after_y_cost
                            if i_to_j_best < delta:
                                i_to_j_best = delta
                                i_to_j_pos = (x, y)

                if len(swap1_route.hist) - swap1_route.line_idx > 2 and len(
                        swap2_route.hist) - swap2_route.line_idx > 2:
                    # back to back
                    for x in range(swap1_route.line_idx, len(swap1_route.hist) - 1):
                        pre_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap1_route.hist[x]] + \
                                     self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]]
                        for y in range(swap2_route.line_idx, len(swap2_route.hist) - 1):
                            if swap2_route.back_load + spool.node_demand[swap1_route.hist[x]] - spool.node_demand[
                                swap2_route.hist[y]] > spool.capa:
                                continue
                            if swap1_route.back_load + spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                                swap1_route.hist[x]] > spool.capa:
                                continue
                            # 개선 비용 = 개선 비용 - 추가 비용
                            pre_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap2_route.hist[y]] + \
                                         self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]]

                            after_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap2_route.hist[y]] + \
                                           self.dist_mat[swap2_route.hist[y]][swap1_route.hist[x + 1]]

                            after_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap1_route.hist[x]] + \
                                           self.dist_mat[swap1_route.hist[x]][swap2_route.hist[y + 1]]

                            # 개선 비용 = 개선 비용 - 추가 비용
                            delta = pre_x_cost + pre_y_cost - after_x_cost - after_y_cost
                            if i_to_j_best < delta:
                                i_to_j_best = delta
                                i_to_j_pos = (x, y)

                self.swap11_status[i, j] = i_to_j_best if i_to_j_best else -1
                self.swap11_status_pos[(i, j)] = i_to_j_pos
        i, j = np.unravel_index(np.argmax(self.swap11_status), self.swap11_status.shape)
        if self.swap11_status[i, j] > 0:
            improved = True
            x, y = self.swap11_status_pos[(i, j)]
            # line_idx 업데이트, load 업데이트, hash 검사
            swap1_hist = current_sol[i].hist
            swap2_hist = current_sol[j].hist
            swap1_hist[x], swap2_hist[y] = swap2_hist[y], swap1_hist[x]
            cost1 = sum(
                self.dist_mat[swap1_hist[i - 1]][swap1_hist[i]] for i in range(1, len(swap1_hist)))
            cost2 = sum(
                self.dist_mat[swap2_hist[i - 1]][swap2_hist[i]] for i in range(1, len(swap2_hist)))
            spool.current_sol[i] = spool.make_sol(swap1_hist, cost1)
            spool.current_sol[j] = spool.make_sol(swap2_hist, cost2)
            # spool.add_pool(swap1_hist, cost1)
            # spool.add_pool(swap2_hist, cost2)

            self.status_update(i, j)

        return improved

    def swap21(self, spool):
        improved = False
        current_sol = spool.current_sol
        len_sol = len(current_sol)
        for i in range(len_sol):
            for j in range(len_sol):
                if i == j or self.swap21_status[i, j] != 0:
                    continue
                swap1_route = current_sol[i]
                swap2_route = current_sol[j]
                i_to_j_best = 0
                i_to_j_pos = 0

                if swap1_route.line_idx > 2 and swap2_route.line_idx > 1:
                    # line to line
                    for x in range(1, swap1_route.line_idx - 1):
                        pre_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap1_route.hist[x]] + \
                                     self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]] + \
                                     self.dist_mat[swap1_route.hist[x + 1]][swap1_route.hist[x + 2]]
                        for y in range(1, swap2_route.line_idx):
                            if swap2_route.line_load + spool.node_demand[swap1_route.hist[x]] + spool.node_demand[
                                swap1_route.hist[x + 1]] - spool.node_demand[
                                swap2_route.hist[y]] > spool.capa:
                                continue
                            if swap1_route.line_load + spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                                swap1_route.hist[x]] - spool.node_demand[
                                swap1_route.hist[x + 1]] > spool.capa:
                                continue

                            pre_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap2_route.hist[y]] + \
                                         self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]]

                            after_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap2_route.hist[y]] + \
                                           self.dist_mat[swap2_route.hist[y]][swap1_route.hist[x + 2]]

                            after_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap1_route.hist[x]] + \
                                           self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]] + \
                                           self.dist_mat[swap1_route.hist[x + 1]][swap2_route.hist[y + 1]]

                            # 개선 비용 = 개선 비용 - 추가 비용
                            delta = pre_x_cost + pre_y_cost - after_x_cost - after_y_cost
                            if i_to_j_best < delta:
                                i_to_j_best = delta
                                i_to_j_pos = (x, y)

                if len(swap1_route.hist) - swap1_route.line_idx > 3 and len(
                        swap2_route.hist) - swap2_route.line_idx > 2:
                    # back to back
                    for x in range(swap1_route.line_idx, len(swap1_route.hist) - 2):
                        pre_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap1_route.hist[x]] + \
                                     self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]]
                        for y in range(swap2_route.line_idx, len(swap2_route.hist) - 1):
                            if swap2_route.back_load + spool.node_demand[swap1_route.hist[x]] + spool.node_demand[
                                swap1_route.hist[x + 1]] - spool.node_demand[
                                swap2_route.hist[y]] > spool.capa:
                                continue
                            if swap1_route.back_load + spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                                swap1_route.hist[x]] - spool.node_demand[
                                swap1_route.hist[x + 1]] > spool.capa:
                                continue

                            pre_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap2_route.hist[y]] + \
                                         self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]]

                            after_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap2_route.hist[y]] + \
                                           self.dist_mat[swap2_route.hist[y]][swap1_route.hist[x + 2]]

                            after_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap1_route.hist[x]] + \
                                           self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]] + \
                                           self.dist_mat[swap1_route.hist[x + 1]][swap2_route.hist[y + 1]]

                            # 개선 비용 = 개선 비용 - 추가 비용
                            delta = pre_x_cost + pre_y_cost - after_x_cost - after_y_cost
                            if i_to_j_best < delta:
                                i_to_j_best = delta
                                i_to_j_pos = (x, y)

                self.swap21_status[i, j] = i_to_j_best if i_to_j_best else -1
                self.swap21_status_pos[(i, j)] = i_to_j_pos
        i, j = np.unravel_index(np.argmax(self.swap21_status), self.swap21_status.shape)
        if self.swap21_status[i, j] > 0:
            improved = True
            x, y = self.swap21_status_pos[(i, j)]
            # line_idx 업데이트, load 업데이트, hash 검사
            swap1_hist = current_sol[i].hist
            swap2_hist = current_sol[j].hist
            swap1_hist[x], swap2_hist[y] = swap2_hist[y], swap1_hist[x]
            v = swap1_hist.pop(x + 1)
            swap2_hist.insert(y + 1, v)

            cost1 = sum(
                self.dist_mat[swap1_hist[i - 1]][swap1_hist[i]] for i in range(1, len(swap1_hist)))
            cost2 = sum(
                self.dist_mat[swap2_hist[i - 1]][swap2_hist[i]] for i in range(1, len(swap2_hist)))
            spool.current_sol[i] = spool.make_sol(swap1_hist, cost1)
            spool.current_sol[j] = spool.make_sol(swap2_hist, cost2)
            # spool.add_pool(swap1_hist, cost1)
            # spool.add_pool(swap2_hist, cost2)

            self.status_update(i, j)
        return improved

    def swap22(self, spool):
        improved = False
        current_sol = spool.current_sol
        len_sol = len(current_sol)
        for i in range(len_sol):
            for j in range(i + 1, len_sol):
                if self.swap22_status[i, j] != 0:
                    continue
                swap1_route = current_sol[i]
                swap2_route = current_sol[j]
                i_to_j_best = 0
                i_to_j_pos = 0

                # line to line
                if swap1_route.line_idx > 2 and swap2_route.line_idx > 2:
                    for x in range(1, swap1_route.line_idx - 1):
                        pre_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap1_route.hist[x]] + \
                                     self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]] + \
                                     self.dist_mat[swap1_route.hist[x + 1]][swap1_route.hist[x + 2]]
                        for y in range(1, swap2_route.line_idx - 1):
                            delta_load = spool.node_demand[swap1_route.hist[x]] + spool.node_demand[
                                swap1_route.hist[x + 1]] - spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                                             swap2_route.hist[y + 1]]
                            if swap2_route.line_load + delta_load > spool.capa:
                                continue
                            if swap1_route.line_load - delta_load > spool.capa:
                                continue

                            pre_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap2_route.hist[y]] + \
                                         self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]] + \
                                         self.dist_mat[swap2_route.hist[y + 1]][swap2_route.hist[y + 2]]

                            after_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap2_route.hist[y]] + \
                                           self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]] + \
                                           self.dist_mat[swap2_route.hist[y + 1]][swap1_route.hist[x + 2]]

                            after_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap1_route.hist[x]] + \
                                           self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]] + \
                                           self.dist_mat[swap1_route.hist[x + 1]][swap2_route.hist[y + 2]]

                            # 개선 비용 = 개선 비용 - 추가 비용
                            delta = pre_x_cost + pre_y_cost - after_x_cost - after_y_cost
                            if i_to_j_best < delta:
                                i_to_j_best = delta
                                i_to_j_pos = (x, y)

                if len(swap1_route.hist) - swap1_route.line_idx > 3 and len(
                        swap2_route.hist) - swap2_route.line_idx > 3:
                    # back to back
                    for x in range(swap1_route.line_idx, len(swap1_route.hist) - 2):
                        pre_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap1_route.hist[x]] + \
                                     self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]]
                        for y in range(swap2_route.line_idx, len(swap2_route.hist) - 2):
                            delta_load = spool.node_demand[swap1_route.hist[x]] + spool.node_demand[
                                swap1_route.hist[x + 1]] - spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                                             swap2_route.hist[y + 1]]
                            if swap2_route.back_load + delta_load > spool.capa:
                                continue
                            if swap1_route.back_load - delta_load > spool.capa:
                                continue

                            pre_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap2_route.hist[y]] + \
                                         self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]] + \
                                         self.dist_mat[swap2_route.hist[y + 1]][swap2_route.hist[y + 2]]

                            after_x_cost = self.dist_mat[swap1_route.hist[x - 1]][swap2_route.hist[y]] + \
                                           self.dist_mat[swap2_route.hist[y]][swap2_route.hist[y + 1]] + \
                                           self.dist_mat[swap2_route.hist[y + 1]][swap1_route.hist[x + 2]]

                            after_y_cost = self.dist_mat[swap2_route.hist[y - 1]][swap1_route.hist[x]] + \
                                           self.dist_mat[swap1_route.hist[x]][swap1_route.hist[x + 1]] + \
                                           self.dist_mat[swap1_route.hist[x + 1]][swap2_route.hist[y + 2]]

                            # 개선 비용 = 개선 비용 - 추가 비용
                            delta = pre_x_cost + pre_y_cost - after_x_cost - after_y_cost
                            if i_to_j_best < delta:
                                i_to_j_best = delta
                                i_to_j_pos = (x, y)

                self.swap22_status[i, j] = i_to_j_best if i_to_j_best else -1
                self.swap22_status_pos[(i, j)] = i_to_j_pos
        i, j = np.unravel_index(np.argmax(self.swap22_status), self.swap22_status.shape)
        if self.swap22_status[i, j] > 0:
            improved = True
            x, y = self.swap22_status_pos[(i, j)]
            # line_idx 업데이트, load 업데이트, hash 검사
            swap1_hist = current_sol[i].hist
            swap2_hist = current_sol[j].hist
            swap1_hist[x], swap2_hist[y] = swap2_hist[y], swap1_hist[x]
            swap1_hist[x + 1], swap2_hist[y + 1] = swap2_hist[y + 1], swap1_hist[x + 1]
            cost1 = sum(
                self.dist_mat[swap1_hist[i - 1]][swap1_hist[i]] for i in range(1, len(swap1_hist)))
            cost2 = sum(
                self.dist_mat[swap2_hist[i - 1]][swap2_hist[i]] for i in range(1, len(swap2_hist)))
            spool.current_sol[i] = spool.make_sol(swap1_hist, cost1)
            spool.current_sol[j] = spool.make_sol(swap2_hist, cost2)
            # spool.add_pool(swap1_hist, cost1)
            # spool.add_pool(swap2_hist, cost2)
            self.status_update(i, j)

        return improved

    def two_opt(self, route):
        result = False
        improved = True
        while improved:
            improved = False
            for i in range(1, route.line_idx - 1):
                for j in range(i + 1, route.line_idx):
                    # before : 기존 경로 / after : 변경된 부분
                    before = (self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[j]][
                        route.hist[j + 1]])
                    after = (self.dist_mat[route.hist[i - 1]][route.hist[j]] + self.dist_mat[route.hist[i]][
                        route.hist[j + 1]])
                    if after - before < 0:
                        route.cost += after - before
                        route.hist = route.hist[:i] + list(reversed(route.hist[i:j + 1])) + route.hist[j + 1:]
                        improved = True
                        result = True

            for i in range(route.line_idx, len(route.hist) - 2):
                for j in range(i + 1, len(route.hist) - 1):
                    before = (self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[j]][
                        route.hist[j + 1]])
                    after = (self.dist_mat[route.hist[i - 1]][route.hist[j]] + self.dist_mat[route.hist[i]][
                        route.hist[j + 1]])
                    if after - before < 0:
                        route.cost += after - before
                        route.hist = route.hist[:i] + list(reversed(route.hist[i:j + 1])) + route.hist[j + 1:]
                        improved = True
                        result = True
        return result

    def reinsertion(self, route):
        result = False
        improved = False
        while improved:
            improved = False
            for i in range(1, route.line_idx):
                save_cost = self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[i]][
                    route.hist[i + 1]] - self.dist_mat[route.hist[i - 1]][route.hist[i + 1]]
                for j in range(i, route.line_idx):
                    if i == j: continue
                    increase_cost = self.dist_mat[route.hist[j]][route.hist[i]] + self.dist_mat[route.hist[i]][
                        route.hist[j + 1]]
                    if save_cost - increase_cost < 0:
                        v = route.hist.pop(i)
                        if i > j:
                            route.hist.insert(j, v)
                        else:
                            route.hist.insert(j - 1, v)
                        route.cost -= save_cost - increase_cost
                        improved = True
                        result = True

            for i in range(route.line_idx, len(route.hist) - 1):
                save_cost = self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[i]][
                    route.hist[i + 1]] - self.dist_mat[route.hist[i - 1]][route.hist[i + 1]]
                for j in range(route.line_idx, len(route.hist) - 1):
                    if i == j: continue
                    increase_cost = self.dist_mat[route.hist[j]][route.hist[i]] + self.dist_mat[route.hist[i]][
                        route.hist[j + 1]]
                    if save_cost - increase_cost < 0:
                        v = route.hist.pop(i)
                        if i > j:
                            route.hist.insert(j, v)
                        else:
                            route.hist.insert(j - 1, v)
                        route.cost -= save_cost - increase_cost
                        improved = True
                        result = True
        return result

    def exchange(self, route):
        result = False
        improved = True
        while improved:
            improved = False
            for i in range(1, route.line_idx - 1):
                for j in range(i + 1, route.line_idx):
                    # before : 기존 경로 / after : 변경된 부분
                    if j - i > 1:
                        before = (self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[i]][
                            route.hist[i + 1]]) + (self.dist_mat[route.hist[j - 1]][route.hist[j]] +
                                                   self.dist_mat[route.hist[j]][
                                                       route.hist[j + 1]])
                        after = (self.dist_mat[route.hist[i - 1]][route.hist[j]] + self.dist_mat[route.hist[j]][
                            route.hist[i + 1]]) + (self.dist_mat[route.hist[j - 1]][route.hist[i]] +
                                                   self.dist_mat[route.hist[i]][
                                                       route.hist[j + 1]])
                    else:
                        before = (self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[j]][
                            route.hist[j + 1]])
                        after = (self.dist_mat[route.hist[i - 1]][route.hist[j]] + self.dist_mat[route.hist[i]][
                            route.hist[j + 1]])
                    if after - before < 0:
                        route.cost += after - before
                        route.hist[i], route.hist[j] = route.hist[j], route.hist[i]
                        improved = True
                        result = True

            for i in range(route.line_idx, len(route.hist) - 2):
                for j in range(i + 1, len(route.hist) - 1):
                    if j - i > 1:
                        before = (self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[i]][
                            route.hist[i + 1]]) + (self.dist_mat[route.hist[j - 1]][route.hist[j]] +
                                                   self.dist_mat[route.hist[j]][
                                                       route.hist[j + 1]])
                        after = (self.dist_mat[route.hist[i - 1]][route.hist[j]] + self.dist_mat[route.hist[j]][
                            route.hist[i + 1]]) + (self.dist_mat[route.hist[j - 1]][route.hist[i]] +
                                                   self.dist_mat[route.hist[i]][
                                                       route.hist[j + 1]])
                    else:
                        before = (self.dist_mat[route.hist[i - 1]][route.hist[i]] + self.dist_mat[route.hist[j]][
                            route.hist[j + 1]])
                        after = (self.dist_mat[route.hist[i - 1]][route.hist[j]] + self.dist_mat[route.hist[i]][
                            route.hist[j + 1]])
                    if after - before < 0:
                        route.cost += after - before
                        route.hist[i], route.hist[j] = route.hist[j], route.hist[i]
                        improved = True
                        result = True
        return result

    def run_rvnd(self, spool):
        inter_nb = [self.shift, self.swap11, self.swap21, self.swap22]
        intra_nb = [self.two_opt, self.reinsertion, self.exchange]
        random.shuffle(inter_nb)
        random.shuffle(intra_nb)

        i = 0
        len_sol = len(spool.current_sol)
        while i < len(inter_nb):
            if inter_nb[i](spool):
                i = 0
            else:
                i += 1
            for k in range(len_sol):
                if self.improved_route[k]:
                    j = 0
                    while j < len(intra_nb):
                        if intra_nb[j](spool.current_sol[k]):
                            j = 0
                        else:
                            j += 1
                    # spool.add_pool(spool.current_sol[k].hist, spool.current_sol[k].cost)
            self.improved_route = [0] * self.K
        spool.calculate_current_cost()
        self.reset_status()

    def perturbation(self, spool, pert_min, pert_max):
        r = random.randint(pert_min, pert_max)
        len_sol = len(spool.current_sol)
        for i in range(r):
            outer_flag = True
            while outer_flag:
                a, b = random.sample(range(len_sol), 2)
                swap1_route, swap2_route = spool.current_sol[a], spool.current_sol[b]
                for x in range(1, swap1_route.line_idx):
                    inner_flag = True
                    for y in range(1, swap2_route.line_idx):
                        if swap2_route.line_load + spool.node_demand[swap1_route.hist[x]] - spool.node_demand[
                            swap2_route.hist[y]] > spool.capa:
                            continue
                        if swap1_route.line_load + spool.node_demand[swap2_route.hist[y]] - spool.node_demand[
                            swap1_route.hist[x]] > spool.capa:
                            continue
                        hist1 = swap1_route.hist
                        hist2 = swap2_route.hist
                        swap1_route.cost -= self.dist_mat[hist1[x - 1]][hist1[x]] + self.dist_mat[hist1[x]][
                            hist1[x + 1]] - (self.dist_mat[hist1[x - 1]][hist2[y]] + self.dist_mat[hist2[y]][
                            hist1[x + 1]])
                        swap2_route.cost -= self.dist_mat[hist2[y - 1]][hist2[y]] + self.dist_mat[hist2[y]][
                            hist2[y + 1]] - (
                                                    self.dist_mat[hist2[y - 1]][hist1[x]] + self.dist_mat[hist1[x]][
                                                hist2[y + 1]])
                        delta_load = spool.node_demand[swap1_route.hist[x]] - spool.node_demand[swap2_route.hist[y]]
                        swap1_route.hist[x], swap2_route.hist[y] = swap2_route.hist[y], swap1_route.hist[x]
                        swap1_route.line_load -= delta_load
                        swap2_route.line_load += delta_load

                        outer_flag = False
                        inner_flag = False
                        break
                    if not inner_flag:
                        break
        spool.calculate_current_cost()

    def run(self, N, spool, solv_SC, start, time_limit=60, Maxiter=2000, log=False):
        end_flag = False
        pert_min, pert_max = int(N * 0.1), int(N * 0.2)
        ub_min, up_max = int(N * 0.2), int(N * 0.3)
        no_improve = 0
        pert_iter = 0
        for i in range(Maxiter):
            if log:
                print(f"ILS iter {i}")
            MaxIterILS = 50
            iterILS = 0
            while iterILS < MaxIterILS:
                self.run_rvnd(spool)
                if time.time() - start > time_limit - 5:
                    end_flag = True
                    break

                # pull update 지점
                for route in spool.current_sol:
                    spool.add_pool(route.hist, route.cost)

                # accept할지 안할지...
                if spool.current_cost < spool.current_best_cost:
                    if log:
                        print("current best 개선", spool.current_best_cost, spool.current_cost)
                    spool.current_best_cost = spool.current_cost
                    spool.current_best_sol = copy.deepcopy(spool.current_sol)
                    iterILS = 0
                else:
                    spool.current_sol = copy.deepcopy(spool.current_best_sol)
                self.perturbation(spool, pert_min, pert_max)

                iterILS += 1

            if spool.current_best_cost < spool.best_cost:
                if log:
                    print("best 개선", spool.best_cost, spool.current_best_cost)
                no_improve = 0
                spool.best_cost = spool.current_best_cost
                spool.best_sol = copy.deepcopy(spool.current_best_sol)
            else:
                no_improve += 1
                pert_iter += 1
                if no_improve >= 10 and pert_iter <= 5:
                    pert_min = min(pert_min + 1, ub_min)
                    pert_max = min(pert_max + 1, up_max)
                    pert_iter = 0

            if end_flag:
                opt_result, obj = solv_SC(spool, self.dist_mat, N, self.K, log=True)
                spool.best_cost = obj
                # print(opt_result)
                spool.best_sol = [spool.make_sol(route, spool.cost_hash[spool.get_hash(route)]) for route in opt_result]
                return

            if not i % 10:
                opt_result, obj = solv_SC(spool, self.dist_mat, N, self.K, log=True)
                if obj < spool.best_cost:
                    no_improve = 0
                    spool.best_cost = obj
                    # print(opt_result)
                    spool.best_sol = [spool.make_sol(route, spool.cost_hash[spool.get_hash(route)]) for route in
                                      opt_result]
                    spool.reset()

                else:
                    if no_improve > 20:
                        spool.current_sol = copy.deepcopy(spool.best_sol)
                        spool.current_cost = spool.best_cost
                        self.perturbation(spool, 2, 5)
                        self.run_rvnd(spool)
                        spool.current_best_sol = copy.deepcopy(spool.current_sol)
                        spool.current_best_cost = spool.current_cost
                    elif no_improve > 40:
                        spool.current_sol = copy.deepcopy(spool.best_sol)
                        spool.current_cost = spool.best_cost
                        self.perturbation(spool, 10, 20)
                        self.run_rvnd(spool)
                        spool.current_best_sol = copy.deepcopy(spool.current_sol)
                        spool.current_best_cost = spool.current_cost
