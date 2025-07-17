import time
from HSH_GRASP2 import generate_grasp_routes
from HSH_Instance import generate_instance
from HSH_Local_Search3 import enhanced_local_search
from typing import List, Dict, Tuple

# 40초 간 반복
def run_pooling_loop(instance: Dict, duration_seconds: int = 40) -> List[List[int]]:
    pool: List[List[int]] = []
    route_set_set: set[Tuple[Tuple[int, ...], ...]] = set()  # 전체 route set 기준 중복 체크

    start_time = time.time()

    while time.time() - start_time < duration_seconds:
        route_set = generate_grasp_routes(instance, alpha=0.3)
        improved_routes = enhanced_local_search(instance, route_set)

        # route set 자체를 key로 변환 (route 순서 정렬하여 중복 방지)
        route_set_key = tuple(sorted(tuple(route) for route in improved_routes))

        if route_set_key not in route_set_set:
            route_set_set.add(route_set_key)
            pool.extend(improved_routes)  # 개별 route를 풀에 추가

    return pool

if __name__ == '__main__':
    from HSH_Instance import generate_instance

    instance = generate_instance(
        num_nodes = 50,
        linehaul_ratio = 0.66,
        capacity = 5000,
        num_vehicles = 6
    )

    pool = run_pooling_loop(instance, duration_seconds = 28)

    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")
    for idx, route in enumerate(pool[:10]):  # 일부 출력
        print(f"Route {idx+1}: {route}")

