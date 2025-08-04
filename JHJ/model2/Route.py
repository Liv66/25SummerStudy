import random
from typing import List, Iterator, Tuple

class Route:
    def __init__(self, problem_info):
        if problem_info is None:
            raise ValueError("Cannot construct a new route for a null problem")
        self.problem_info = problem_info

        # 라인홀/백홀 고객 목록(노드 인덱스)
        self.linehaul_customers = []
        self.backhaul_customers = []
        self.delivery_load = 0  # sum of linehaul loads
        self.pickup_load = 0    # sum of backhaul loads

    def add_customer(self, node, index = None):
        """
        고객 추가:
        - index None: 자동 위치 (라인홀은 마지막 라인홀 다음, 백홀은 마지막 직전)
        - index i: 해당 위치 (1~size-2) 기준 삽입
        """
        # 삽입 위치 결정
        if index is None:
            # linehual(1)이면 '마지막 linehual 다음(+1) 위치'
            # backhual(2)이면 '복귀 depot 직전(size-1) 위치'
            if self.problem_info['node_types'][node] == 1:  # linehaul
                idx = self.get_linehaul_count() + 1
            else:                                         # backhaul
                idx = self.size() - 1
        else:
            idx = index

        # 인덱스 유효성 검사
        if idx <= 0 or idx >= self.size():
            raise IndexError(f"Cannot add customer at index {idx}")

        # linehual인지 backhaul인지에 따라 분기
        if self.problem_info['node_types'][node] == 1:
            self._add_linehaul(node, idx)
        else:
            self._add_backhaul(node, idx)

    def force_add_customer(self, node, index=None):
        """
        용량 제약을 무시하고 강제로 고객을 추가합니다.
        초기해 생성 시 제약 완화를 위해 사용됩니다.
        """
        # 삽입 위치 결정
        if index is None:
            if self.problem_info['node_types'][node] == 1:  # linehaul
                idx = self.get_linehaul_count() + 1
            else:                                         # backhaul
                idx = self.size() - 1
        else:
            idx = index

        # 인덱스 유효성 검사
        if idx <= 0 or idx >= self.size():
            raise IndexError(f"Cannot add customer at index {idx}")

        # 용량 검사를 건너뛰고 강제로 추가
        if self.problem_info['node_types'][node] == 1:
            self._force_add_linehaul(node, idx)
        else:
            self._force_add_backhaul(node, idx)

    # 고객을 해당 route에 추가
    def _add_linehaul(self, node, idx):
        capa = self.problem_info['capa']
        load = self.problem_info['node_demands'][node]

        # 라인홀 리스트에 삽입 (idx-1 위치)
        pos = idx - 1
        if pos < 0 or pos > len(self.linehaul_customers):
            raise IndexError(f"Invalid linehaul position {pos}")

        # insert + load 갱신
        self.linehaul_customers.insert(pos, node)
        self.delivery_load += load

    def _add_backhaul(self, node, idx):
        capa = self.problem_info['capa']
        load = self.problem_info['node_demands'][node]

        if self.get_linehaul_count() == 0:
            raise ValueError("라인홀 고객 없이 백홀만으로 경로를 구성할 수 없습니다.")

        # 순서
        line_count = self.get_linehaul_count()
        if idx <= line_count:
            raise ValueError(f"Backhaul must be after linehaul, invalid index {idx}")

        # 실제 삽입 위치: idx - line_count - 1
        pos = idx - line_count - 1
        if pos < 0 or pos > len(self.backhaul_customers):
            raise IndexError(f"Invalid backhaul position {pos}")
        self.backhaul_customers.insert(pos, node)
        self.pickup_load += load

    def _force_add_linehaul(self, node, idx):
        """용량 검사 없이 라인홀 고객 강제 추가"""
        load = self.problem_info['node_demands'][node]
        pos = idx - 1
        if pos < 0 or pos > len(self.linehaul_customers):
            raise IndexError(f"Invalid linehaul position {pos}")
        self.linehaul_customers.insert(pos, node)
        self.delivery_load += load

    def _force_add_backhaul(self, node, idx):
        """용량 검사 없이 백홀 고객 강제 추가"""
        load = self.problem_info['node_demands'][node]
        
        # 백홀만으로 경로 구성 방지 검사는 유지
        if self.get_linehaul_count() == 0:
            raise ValueError("라인홀 고객 없이 백홀만으로 경로를 구성할 수 없습니다.")
            
        # 순서 검사는 유지
        line_count = self.get_linehaul_count()
        if idx <= line_count:
            raise ValueError(f"Backhaul must be after linehaul, invalid index {idx}")

        pos = idx - line_count - 1
        if pos < 0 or pos > len(self.backhaul_customers):
            raise IndexError(f"Invalid backhaul position {pos}")
        self.backhaul_customers.insert(pos, node)
        self.pickup_load += load

    def get(self, i) -> int:
        """i: 0..size-1, 0 or size-1은 depot (0) 반환"""
        if i <= 0 or i >= self.size() - 1:
            return 0
        # i in 1..line_count
        lc = self.get_linehaul_count()
        if i <= lc:
            return self.linehaul_customers[i-1]
        # backhaul
        return self.backhaul_customers[i - lc - 1]

    def remove_customer(self, i):
        if i <= 0 or i >= self.size() - 1:
            raise IndexError(f"Cannot remove customer at index {i}")
        lc, bc = self.get_linehaul_count(), self.get_backhaul_count()
        if i <= lc:
            node = self.linehaul_customers.pop(i-1)
            self.delivery_load -= self.problem_info['node_demands'][node]
        else:
            node = self.backhaul_customers.pop(i - lc - 1)
            self.pickup_load -= self.problem_info['node_demands'][node]

    def set_customer(self, node, i):
        # 교체: remove + add
        self.remove_customer(i)
        self.add_customer(node, i)

    def shuffle(self):
        random.shuffle(self.linehaul_customers)
        random.shuffle(self.backhaul_customers)

    def get_distance_cost(self) -> float:
        dist = self.problem_info['dist_mat']
        cost = 0.0
        prev = 0
        for i in range(self.size() - 1):
            cur = self.get(i)
            nxt = self.get(i+1)
            cost += dist[cur][nxt]
        return cost

    def get_penalty_cost(self, penalty_rate: float) -> float:
        """경로의 용량 위반에 대한 페널티 비용을 계산합니다."""
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
        """이 경로의 최대 용량을 반환합니다."""
        return self.problem_info['capa']

    def get_customers(self):
        return self.linehaul_customers + self.backhaul_customers

    def get_linehaul_customers(self):
        return list(self.linehaul_customers)

    def get_backhaul_customers(self):
        return list(self.backhaul_customers)

    def get_linehaul_count(self):
        return len(self.linehaul_customers)

    def get_backhaul_count(self):
        return len(self.backhaul_customers)

    def size(self):
        return self.get_linehaul_count() + self.get_backhaul_count() + 2

    def __str__(self) -> str:
        customers = [str(n) for n in self.get_customers()]
        return "0 " + " ".join(str(n) for n in self.get_customers()) + " 0"

    def __eq__(self, other) -> bool:
        if not isinstance(other, Route):
            return False
        return (self.linehaul_customers == other.linehaul_customers and
                self.backhaul_customers == other.backhaul_customers)

    def __hash__(self) -> int:
        return hash((tuple(self.linehaul_customers), tuple(self.backhaul_customers)))

    def __iter__(self) -> Iterator[int]:
        # depot -> linehaul -> backhaul -> depot
        yield 0
        for n in self.get_customers():
            yield n
        yield 0
