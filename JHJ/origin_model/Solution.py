from __future__ import annotations
from typing import List
from Route import Route

class CVRPBSolution:

    def __init__(self, routes):
        self.routes = routes
        self._total_cost = None
        self.penalty_rate = 10000  # 제약 위반에 대한 강력한 페널티

    def get_routes(self):
        return self.routes
        
    def get_problem_info(self) -> dict:
        if not self.routes:
            # 경로가 없는 경우, 어떻게 처리할지 결정해야 함.
            # 예를 들어, 빈 dict를 반환하거나 예외를 발생시킬 수 있습니다.
            # 여기서는 첫 번째 경로의 정보를 대표로 사용한다고 가정합니다.
            raise ValueError("Cannot get problem info from a solution with no routes.")
        return self.routes[0].problem_info


    def get_total_cost(self):
        if self._total_cost is None:
            distance_cost = sum(r.get_distance_cost() for r in self.routes)
            penalty_cost = self.get_penalty_cost(self.penalty_rate)
            self._total_cost = distance_cost + penalty_cost
        return self._total_cost

    def get_penalty_cost(self, penalty_rate):
        # 해 전체 패널티 반환
        return sum(r.get_penalty_cost(penalty_rate) for r in self.routes)
    
    
    def update_cost_with_gain(self, gain):
        # 계산된 이득(gain)을 사용하여 총비용을 직접 업데이트

        if self._total_cost is not None:
            self._total_cost -= gain
        # 만약 _total_cost가 None이면, 다음 get_total_cost() 호출 시 어차피 재계산
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

