# GeneratePool - Pool 생성, 관리
import random, time, json
from typing import List, Dict

from HSH.HSH_GRASP import generate_grasp_routes
from HSH.HSH_TS import tabu_search

class SolPool:
    """
    random_cost: 각 노드에 부여된 난수 가중치 (depot은 0)
    route_hash: 노드 합(순서 무시)
    pool: 고유 route_hash를 담는 list
    sol_hash: route_hash -> 해당 해시의 최소 비용 경로
    cost_hash: route_hash -> 현재까지의 최소 비용
    """
    def __init__(self, N: int):
        depot = 0
        seed = 42
        self.depot = depot
        rng = random.Random(seed)
        # depot은 0, 나머지는 [0,1) 난수
        self.random_cost: List[float] = [
            0.0 if i == depot else rng.random() for i in range(N)
        ]

        self.pool: List[float] = []                 # 고유 해시 목록
        self.sol_hash: Dict[float, List[int]] = {}  # 해시 -> 경로
        self.cost_hash: Dict[float, float] = {}     # 해시 -> 비용

    def get_hash(self, route: List[int]) -> float:
        # 경로 내부 노드의 random_cost 합으로 해시 생성(순서 무시)
        inner = route[1:-1]
        return float(sum(self.random_cost[n] for n in inner))

    @staticmethod
    def route_cost(route: List[int], dist_mat: List[List[float]]) -> float:
        return sum(dist_mat[route[i]][route[i + 1]] for i in range(len(route) - 1))

    def add_pool(self, route: List[int], dist_mat: List[List[float]]) -> None:
        """
        1. route_hash가 처음이면 pool에 해시 추가 + sol_hash/cost_hash 신규 등록
        2. 이미 존재하면 pool엔 아무것도 하지 않음. 비용이 더 적으면 경로/비용만 갱신
        """
        h = self.get_hash(route)
        c = self.route_cost(route, dist_mat)

        if h in self.sol_hash:
            # 방문 순서만 개선된 동일 노드 구성 → 비용/경로만 갱신, pool 변화 없음
            if c < self.cost_hash[h]:
                self.cost_hash[h] = c
                self.sol_hash[h] = route.copy()
            return

        # 최초 등장 해시 -> pool/sol_hash/cost_hash에 등록
        self.sol_hash[h] = route.copy()
        self.cost_hash[h] = c
        self.pool.append(h)

    def export_pool_routes(self) -> List[List[int]]:
        return [self.sol_hash[h] for h in self.pool]


def run_pooling_loop(N, K, node_type, node_demand, capa, dist_mat) -> List[List[int]]:
    depot = 0
    sp = SolPool(N=N)
    duration_seconds = int(50)

    t0 = time.time()
    while time.time() - t0 < duration_seconds:
        route_set = generate_grasp_routes(N, K, node_type, node_demand, capa, dist_mat)
        improved_routes = tabu_search(N, node_type, node_demand, capa, dist_mat, route_set, max_iter=100, tabu_tenure=13)

        for route in improved_routes:
            sp.add_pool(route, dist_mat)

    return sp.export_pool_routes()

if __name__ == '__main__':
    random.seed(42)
    with open(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_130_0.85.json", encoding="utf-8") as f:
        problem_info = json.load(f)
    N = problem_info['N']
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = [abs(d) for d in problem_info["node_demands"]]
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    pool = run_pooling_loop(N, K, node_type, node_demand, capa, dist_mat)

    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")
    for idx, route in enumerate(pool[:10]):
        print(f"Route {idx+1}: {route}")
