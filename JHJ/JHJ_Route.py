import random
from typing import List, Iterator, Tuple


class Route:
    def __init__(self, problem_info):
        self.problem_info = problem_info
        self.linehaul_customers = []
        self.backhaul_customers = []
        self.delivery_load = 0
        self.pickup_load = 0

    def add_customer(self, node, index):
        if not (0 < index < self.size()):
            if not (isinstance(node, int) and index == self.size()):
                raise IndexError(f"Cannot add customer at index {index}")

        if self.problem_info['node_types'][node] == 1:
            self._add_linehaul(node, index)
        else:
            self._add_backhaul(node, index)

    def force_add_customer(self, node):
        if self.problem_info['node_types'][node] == 1:
            pos = len(self.linehaul_customers)
            self.linehaul_customers.insert(pos, node)
            self.delivery_load += self.problem_info['node_demands'][node]
        else:
            pos = len(self.backhaul_customers)
            self.backhaul_customers.insert(pos, node)
            self.pickup_load += self.problem_info['node_demands'][node]

    def _add_linehaul(self, node, idx):
        load = self.problem_info['node_demands'][node]
        pos = idx - 1
        if not (0 <= pos <= len(self.linehaul_customers)):
            raise IndexError(f"Invalid linehaul position {pos}")
        self.linehaul_customers.insert(pos, node)
        self.delivery_load += load

    def _add_backhaul(self, node, idx):
        load = self.problem_info['node_demands'][node]
        if self.get_linehaul_count() == 0:
            raise ValueError("라인홀 고객 없이 백홀만으로 경로를 구성할 수 없습니다.")
        line_count = self.get_linehaul_count()
        if idx <= line_count:
            raise ValueError(f"Backhaul must be after linehaul, invalid index {idx}")
        pos = idx - line_count - 1
        if not (0 <= pos <= len(self.backhaul_customers)):
            raise IndexError(f"Invalid backhaul position {pos}")
        self.backhaul_customers.insert(pos, node)
        self.pickup_load += load

    def get(self, i) -> int:
        if i <= 0 or i >= self.size() - 1:
            return 0
        lc = self.get_linehaul_count()
        if i <= lc:
            return self.linehaul_customers[i - 1]
        return self.backhaul_customers[i - lc - 1]

    def remove_customer(self, i):
        if not (0 < i < self.size() - 1):
            raise IndexError(f"Cannot remove customer at index {i}")
        lc = self.get_linehaul_count()
        if i <= lc:
            node = self.linehaul_customers.pop(i - 1)
            self.delivery_load -= self.problem_info['node_demands'][node]
        else:
            node = self.backhaul_customers.pop(i - lc - 1)
            self.pickup_load -= self.problem_info['node_demands'][node]

    def remove_customer_by_id(self, customer_id):
        try:
            self.linehaul_customers.remove(customer_id)
            self.delivery_load -= self.problem_info['node_demands'][customer_id]
            return
        except ValueError:
            pass  # 라인홀에 없으면 백홀에서 찾음
        try:
            self.backhaul_customers.remove(customer_id)
            self.pickup_load -= self.problem_info['node_demands'][customer_id]
            return
        except ValueError:
            raise ValueError(f"Customer {customer_id} not found in this route.")

    def set_customers(self, customers: list, problem_info: dict):
        self.linehaul_customers = []
        self.backhaul_customers = []
        self.delivery_load = 0
        self.pickup_load = 0

        node_types = problem_info['node_types']
        node_demands = problem_info['node_demands']

        for customer in customers:
            if node_types[customer] == 1:
                self.linehaul_customers.append(customer)
                self.delivery_load += node_demands[customer]
            else:
                self.backhaul_customers.append(customer)
                self.pickup_load += node_demands[customer]

    def get_distance_cost(self):
        dist = self.problem_info['dist_mat']
        cost = 0.0
        prev = 0
        for i in range(self.size()):
            cur = self.get(i)
            cost += dist[prev][cur]
            prev = cur
        cost += dist[prev][0]  # 마지막 고객에서 depot으로 복귀
        return cost

    def get_cost_increase_for_insertion(self, customer, index, penalty_rate):

        dist = self.problem_info['dist_mat']
        demands = self.problem_info['node_demands']

        # 1. 거리 비용 증가분 계산
        prev_node = self.get(index - 1)
        next_node = self.get(index)
        distance_increase = dist[prev_node][customer] + dist[customer][next_node] - dist[prev_node][next_node]

        # 2. 페널티 비용 증가분 계산
        penalty_before = self.get_penalty_cost(penalty_rate)

        is_linehaul = self.problem_info['node_types'][customer] == 1
        customer_demand = demands[customer]

        # 가상 삽입 후의 부하량
        delivery_load_after = self.delivery_load + (customer_demand if is_linehaul else 0)
        pickup_load_after = self.pickup_load + (0 if is_linehaul else customer_demand)

        capacity = self.get_capacity()
        violation_after = max(0, delivery_load_after - capacity) + max(0, pickup_load_after - capacity)
        penalty_after = violation_after * penalty_rate

        penalty_increase = penalty_after - penalty_before

        return distance_increase + penalty_increase

    def get_penalty_cost(self, penalty_rate):
        penalty = 0.0
        capacity = self.get_capacity()
        linehaul_violation = max(0, self.get_delivery_load() - capacity)
        backhaul_violation = max(0, self.get_pickup_load() - capacity)
        penalty += (linehaul_violation + backhaul_violation) * penalty_rate
        return penalty

    def get_delivery_load(self):
        return self.delivery_load

    def get_pickup_load(self):
        return self.pickup_load

    def get_capacity(self) -> float:
        return self.problem_info['capa']

    def get_customers(self):
        return self.linehaul_customers + self.backhaul_customers

    def get_customers_count(self) -> int:
        return len(self.linehaul_customers) + len(self.backhaul_customers)

    def get_linehaul_customers(self):
        return list(self.linehaul_customers)

    def get_backhaul_customers(self):
        return list(self.backhaul_customers)

    def get_linehaul_count(self):
        return len(self.linehaul_customers)

    def get_backhaul_count(self):
        return len(self.backhaul_customers)

    def size(self):
        return len(self.linehaul_customers) + len(self.backhaul_customers) + 2

    def __iter__(self):
        yield 0
        for n in self.linehaul_customers: yield n
        for n in self.backhaul_customers: yield n
        yield 0
