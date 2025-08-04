from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Solution import CVRPBSolution
    from Route import Route


class CVRPBSolutionNodeIterator:
    """
    CVRPBSolution 내 모든 고객 노드를 순회하는 반복자(Iterator).
    depot은 건너뛰고 고객 노드만 방문합니다.
    """
    def __init__(self, solution: CVRPBSolution):
        if solution is None:
            raise ValueError("Solution cannot be null")
        self.solution = solution
        self.route_iter = iter(solution.get_routes())
        self.current_route = None
        self.customer_index = 0
        self.current_node_index = 0 # Explicitly store the current node index
        self._advance_route()

    def _advance_route(self):
        self.current_route = next(self.route_iter, None)
        while self.current_route and self.current_route.get_linehaul_count() + self.current_route.get_backhaul_count() == 0:
            self.current_route = next(self.route_iter, None)
        self.customer_index = 1 # Start from the first customer

    def has_next(self) -> bool:
        if self.current_route is None:
            return False
        return self.customer_index <= (self.current_route.get_linehaul_count() + self.current_route.get_backhaul_count())

    def next(self) -> int:
        if not self.has_next():
            raise StopIteration
        
        self.current_node_index = self.customer_index
        node = self.current_route.get(self.current_node_index)
        self.customer_index += 1

        if self.customer_index > (self.current_route.get_linehaul_count() + self.current_route.get_backhaul_count()):
            self._advance_route()
            
        return node

    def last_route(self) -> Route:
        return self.current_route

    def last_node_index(self) -> int:
        return self.current_node_index


class RelocateMove:
    def __init__(self, solution: CVRPBSolution, from_iter: CVRPBSolutionNodeIterator, to_iter: CVRPBSolutionNodeIterator):
        if from_iter is None or to_iter is None:
            raise ValueError("Iterators cannot be null")
        self.solution = solution
        self.from_route = from_iter.last_route()
        self.from_index = from_iter.last_node_index()
        self.customer = self.from_route.get(self.from_index)
        self.to_route = to_iter.last_route()
        self.to_index = to_iter.last_node_index()
        self.problem_info = self.from_route.problem_info

    def gain(self) -> float:
        dist = self.problem_info['dist_mat']
        
        # 1. 거리 비용 변화 계산
        prev_from = self.from_route.get(self.from_index - 1)
        next_from = self.from_route.get(self.from_index + 1)
        prev_to = self.to_route.get(self.to_index - 1)
        next_to = self.to_route.get(self.to_index)
        
        pre_cost = (dist[prev_from][self.customer] + dist[self.customer][next_from] + dist[prev_to][next_to])
        post_cost = (dist[prev_from][next_from] + dist[prev_to][self.customer] + dist[self.customer][next_to])
        distance_gain = pre_cost - post_cost
        
        # 2. 페널티 비용 변화 계산
        penalty_rate = self.solution.penalty_rate
        penalty_before = self.from_route.get_penalty_cost(penalty_rate) + self.to_route.get_penalty_cost(penalty_rate)
        
        demands = self.problem_info['node_demands']
        customer_demand = demands[self.customer]
        types = self.problem_info['node_types']
        is_linehaul = (types[self.customer] == 1)
        
        from_delivery_load_after = self.from_route.get_delivery_load() - (customer_demand if is_linehaul else 0)
        from_pickup_load_after = self.from_route.get_pickup_load() - (0 if is_linehaul else customer_demand)
        to_delivery_load_after = self.to_route.get_delivery_load() + (customer_demand if is_linehaul else 0)
        to_pickup_load_after = self.to_route.get_pickup_load() + (0 if is_linehaul else customer_demand)

        from_violation_after = max(0, from_delivery_load_after - self.from_route.get_capacity()) + max(0, from_pickup_load_after - self.from_route.get_capacity())
        to_violation_after = max(0, to_delivery_load_after - self.to_route.get_capacity()) + max(0, to_pickup_load_after - self.to_route.get_capacity())
        
        penalty_after = (from_violation_after + to_violation_after) * penalty_rate
        penalty_gain = penalty_before - penalty_after

        return distance_gain + penalty_gain


    def is_legal(self) -> bool:
        if self.to_index == 0:
            return False
        return self._check_order()


    def _check_order(self) -> bool:
        types = self.problem_info['node_types']
        is_linehaul = (types[self.customer] == 1)

        # A backhaul customer cannot be moved to a route with no linehaul customers
        if not is_linehaul and self.from_route is not self.to_route and self.to_route.get_linehaul_count() == 0:
            return False

        if is_linehaul and self.from_route.get_linehaul_count() == 1 and self.from_route.get_backhaul_count() > 0:
            return False

        # Correctly determine valid insertion range
        if is_linehaul:
            # Can be inserted anywhere among other linehauls
            return self.to_index <= self.to_route.get_linehaul_count() + 1
        else: # is_backhaul
            # Can be inserted anywhere among other backhauls
            return self.to_index > self.to_route.get_linehaul_count()

    def apply(self):
        self.from_route.remove_customer(self.from_index)
        
        i = self.to_index
        if self.from_route is self.to_route and self.from_index < self.to_index:
            i -= 1
        self.to_route.add_customer(self.customer, i)


class ExchangeMove:
    def __init__(self, solution: CVRPBSolution, first_iter: CVRPBSolutionNodeIterator, second_iter: CVRPBSolutionNodeIterator):
        if first_iter is None or second_iter is None:
            raise ValueError("Iterators cannot be null")
        self.solution = solution
        self.first_route = first_iter.last_route()
        self.first_index = first_iter.last_node_index()
        self.first_customer = self.first_route.get(self.first_index)
        self.second_route = second_iter.last_route()
        self.second_index = second_iter.last_node_index()
        self.second_customer = self.second_route.get(self.second_index)
        self.problem_info = self.first_route.problem_info

    def gain(self) -> float:
        dist = self.problem_info['dist_mat']

        # 1. 거리 비용 변화 계산
        prev1 = self.first_route.get(self.first_index - 1)
        next1 = self.first_route.get(self.first_index + 1)
        prev2 = self.second_route.get(self.second_index - 1)
        next2 = self.second_route.get(self.second_index + 1)

        pre_cost = (dist[prev1][self.first_customer] + dist[self.first_customer][next1] +
                    dist[prev2][self.second_customer] + dist[self.second_customer][next2])
        
        post_cost = (dist[prev1][self.second_customer] + dist[self.second_customer][next1] +
                     dist[prev2][self.first_customer] + dist[self.first_customer][next2])

        if self.first_route is self.second_route and abs(self.first_index - self.second_index) == 1:
            pre_cost -= 2 * dist[self.first_customer][self.second_customer]
            
        distance_gain = pre_cost - post_cost

        # 2. 페널티 비용 변화 계산
        penalty_rate = self.solution.penalty_rate
        penalty_before = self.first_route.get_penalty_cost(penalty_rate) + self.second_route.get_penalty_cost(penalty_rate)
        
        demands = self.problem_info['node_demands']
        demand1 = demands[self.first_customer]
        demand2 = demands[self.second_customer]
        types = self.problem_info['node_types']
        is_linehaul = (types[self.first_customer] == 1)

        load_after1 = (self.first_route.get_delivery_load() if is_linehaul else self.first_route.get_pickup_load()) - demand1 + demand2
        load_after2 = (self.second_route.get_delivery_load() if is_linehaul else self.second_route.get_pickup_load()) - demand2 + demand1
        
        violation_after1 = max(0, load_after1 - self.first_route.get_capacity())
        violation_after2 = max(0, load_after2 - self.second_route.get_capacity())

        penalty_after = (violation_after1 + violation_after2) * penalty_rate
        penalty_gain = penalty_before - penalty_after

        return distance_gain + penalty_gain


    def is_legal(self) -> bool:
        return self._check_order()


    def _check_order(self) -> bool:
        types = self.problem_info['node_types']
        # Exchange is only feasible between customers of the same type.
        if types[self.first_customer] != types[self.second_customer]:
            return False
        
        # If the move is within the same route, it's always valid order-wise.
        if self.first_route is self.second_route:
            return True
            
        # If moving between routes, the precedence constraint (linehaul before backhaul)
        # for each route must be maintained. Since we only swap same-type customers,
        # we only need to ensure we are not creating an empty linehaul section in a route
        # that has backhaul customers.
        if types[self.first_customer] == 1: # Both are linehaul
            if self.first_route.get_linehaul_count() == 1 and self.first_route.get_backhaul_count() > 0:
                return False
            if self.second_route.get_linehaul_count() == 1 and self.second_route.get_backhaul_count() > 0:
                return False
        
        return True


    def apply(self):
        c1, c2 = self.first_customer, self.second_customer
        
        # To avoid issues with replacing and recalculating loads, just set them
        self.first_route.set_customer(c2, self.first_index)
        self.second_route.set_customer(c1, self.second_index)
