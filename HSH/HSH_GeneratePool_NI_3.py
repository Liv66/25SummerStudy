import time
import random
from typing import List, Dict, Tuple

from HSH_GRASP_NI import generate_grasp_routes
from HSH_Local_Search_NI_4 import tabu_search
from HSH_loader import load_instance_from_json

# default: 60
def run_pooling_loop(instance: Dict, duration_seconds: int = 60) -> List[List[int]]:
    pool: List[List[int]] = []
    route_set_set: set[Tuple[Tuple[int, ...], ...]] = set()  # 전체 route set 기준 중복 체크

    start_time = time.time()

    while time.time() - start_time < duration_seconds:
        route_set = generate_grasp_routes(instance, alpha=0.3)
        improved_routes = tabu_search(instance, route_set, max_iter=150, tabu_tenure=7)

        # 중복 제거를 위한 key 생성
        route_set_key = tuple(sorted(tuple(route) for route in improved_routes))

        if route_set_key not in route_set_set:
            route_set_set.add(route_set_key)
            pool.extend(improved_routes)  # 개별 route 추가

    return pool

if __name__ == '__main__':
    random.seed(42)
    instance = load_instance_from_json(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_100_0.7.json")
    instance["node_demands"] = [abs(d) for d in instance["node_demands"]]
    pool = run_pooling_loop(instance, duration_seconds = 5)

    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")
    for idx, route in enumerate(pool[:10]):  # 일부 출력
        print(f"Route {idx+1}: {route}")