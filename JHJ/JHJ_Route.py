import random
from typing import List, Iterator, Tuple


class Route:
    def __init__(self, problem_info):
        self.problem_info = problem_info
        self.linehaul_customers = []
        self.backhaul_customers = []
        self.delivery_load = 0
        self.pickup_load = 0

    def size(self):
        return len(self.linehaul_customers) + len(self.backhaul_customers) + 2

    def add_customer(self, node, idx):

        if self.problem_info['node_types'][node] == 1:
            load = self.problem_info['node_demands'][node]
            pos = idx - 1

            self.linehaul_customers.insert(pos, node)
            self.delivery_load += load
        else:
            load = self.problem_info['node_demands'][node]

            line_count = len(self.linehaul_customers)

            pos = idx - line_count - 1

            self.backhaul_customers.insert(pos, node)
            self.pickup_load += load

    def force_add_customer(self, node):
        if self.problem_info['node_types'][node] == 1:
            pos = len(self.linehaul_customers)
            self.linehaul_customers.insert(pos, node)
            self.delivery_load += self.problem_info['node_demands'][node]
        else:
            pos = len(self.backhaul_customers)
            self.backhaul_customers.insert(pos, node)
            self.pickup_load += self.problem_info['node_demands'][node]


    def get(self, i):
        if i <= 0 or i >= self.size() - 1:
            return 0
        lc = len(self.linehaul_customers)
        if i <= lc:
            return self.linehaul_customers[i - 1]
        return self.backhaul_customers[i - lc - 1]

    def remove_customer(self, i):

        lc = len(self.linehaul_customers)
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

    def set_customers(self, customers, problem_info):
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

        capacity = self.problem_info['capa']
        violation_after = max(0, delivery_load_after - capacity) + max(0, pickup_load_after - capacity)
        penalty_after = violation_after * penalty_rate

        penalty_increase = penalty_after - penalty_before

        return distance_increase + penalty_increase

    def get_penalty_cost(self, penalty_rate):
        penalty = 0.0
        capacity = self.problem_info['capa']
        linehaul_violation = max(0, self.delivery_load - capacity)
        backhaul_violation = max(0, self.pickup_load - capacity)
        penalty += (linehaul_violation + backhaul_violation) * penalty_rate
        return penalty

    def get_customers(self):
        return self.linehaul_customers + self.backhaul_customers

    def get_linehaul_customers(self):
        return list(self.linehaul_customers)

    def get_backhaul_customers(self):
        return list(self.backhaul_customers)

    def __iter__(self):
        yield 0
        for n in self.linehaul_customers: yield n
        for n in self.backhaul_customers: yield n
        yield 0
