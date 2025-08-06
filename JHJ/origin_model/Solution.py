from __future__ import annotations
from typing import List
from Route import Route

class CVRPBSolution:
    """
    하나의 해(solution)를 나타내는 클래스.
    여러 개의 경로(Route) 객체를 포함하며, 해 전체의 비용을 관리합니다.
    """
    def __init__(self, routes):
        if routes is None:
            raise ValueError("Routes list cannot be null.")
        self.routes = routes
        self._total_cost = None
        self.penalty_rate = 10000  # 제약 위반에 대한 강력한 페널티

    def get_routes(self):
        """해에 포함된 모든 경로의 리스트를 반환합니다."""
        return self.routes
        
    def get_problem_info(self) -> dict:
        """문제 인스턴스 정보를 반환합니다."""
        if not self.routes:
            # 경로가 없는 경우, 어떻게 처리할지 결정해야 함.
            # 예를 들어, 빈 dict를 반환하거나 예외를 발생시킬 수 있습니다.
            # 여기서는 첫 번째 경로의 정보를 대표로 사용한다고 가정합니다.
            raise ValueError("Cannot get problem info from a solution with no routes.")
        return self.routes[0].problem_info # problem_info (json) 내뱉음


    def get_total_cost(self):
        """해의 총비용 (거리 비용 + 페널티)을 계산하여 반환합니다."""
        if self._total_cost is None:
            distance_cost = sum(r.get_distance_cost() for r in self.routes)
            penalty_cost = self.get_penalty_cost(self.penalty_rate)
            self._total_cost = distance_cost + penalty_cost
        return self._total_cost

    def get_penalty_cost(self, penalty_rate):
        """해 전체의 페널티 비용을 계산하여 반환합니다."""
        return sum(r.get_penalty_cost(penalty_rate) for r in self.routes)
    
    
    def update_cost_with_gain(self, gain: float):
        """
        계산된 이득(gain)을 사용하여 총비용을 직접 업데이트합니다.
        전체 비용을 다시 계산하는 것을 방지하여 성능을 향상시킵니다.
        """
        if self._total_cost is not None:
            self._total_cost -= gain
        # 만약 _total_cost가 None이면, 다음 get_total_cost() 호출 시 어차피 재계산되므로 안전합니다.

    def invalidate(self):
        """해의 비용 캐시를 무효화하여 다음번에 재계산하도록 합니다."""
        self._total_cost = None


    def get_customers(self) -> list:
        """해에 포함된 모든 고객의 리스트를 반환합니다."""
        all_customers = []
        for route in self.routes:
            all_customers.extend(route.get_customers())
        return all_customers

    def __iter__(self):
        """경로 리스트에 대한 iterator를 반환합니다."""
        return iter(self.routes)

    def __len__(self):
        """해에 포함된 경로의 수를 반환합니다."""
        return len(self.routes)

    def __str__(self):
        """해를 문자열로 표현합니다."""
        return '\n'.join(str(r) for r in self.routes)
