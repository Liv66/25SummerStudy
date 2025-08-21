from __future__ import annotations
from JHJ_Route import Route

class CVRPBSolution:

    def __init__(self, routes):
        self.routes = routes
        self._total_cost = None
        self.penalty_rate = 10000  # 제약 위반에 대한 강력한 페널티


    def get_routes(self):
        return self.routes
        
    def get_problem_info(self):
        return self.routes[0].problem_info


    def get_total_cost(self):
        if self._total_cost is None:
            distance_cost = sum(r.get_distance_cost() for r in self.routes)
            penalty_cost = self.get_penalty_cost(self.penalty_rate)
            self._total_cost = distance_cost + penalty_cost
        return self._total_cost

    def get_penalty_cost(self, penalty_rate):
        return sum(r.get_penalty_cost(penalty_rate) for r in self.routes)
    
    
    def update_cost_with_gain(self, gain):
        if self._total_cost is not None:
            self._total_cost -= gain
    def invalidate(self):
        self._total_cost = None


    def get_customers(self):
        all_customers = []
        for route in self.routes:
            all_customers.extend(route.get_customers())
        return all_customers

    def __iter__(self):
        return iter(self.routes)

    def __len__(self):
        return len(self.routes)

