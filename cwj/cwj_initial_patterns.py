import random
from cwj_pricing_problem import labeling_algorithm

def generate_initial_patterns(K, Q, node_types, node_demands, dist_mat):
    """
    linehaul 여러 개 + backhaul 조합 기반으로 feasible route 생성
    - 중복 경로 제거 (순서 동일)
    - 노드 집합 동일한 경로 중 최소 비용만 유지
    """
    depot = 0
    N = len(node_types)
    linehauls = [i for i in range(1, N) if node_types[i] == 1]
    backhauls = [i for i in range(1, N) if node_types[i] == 2]

    best_by_node_set = dict()       # key: frozenset(route[1:-1]), value: (route, cost)
    unique_route_set = set()        # key: tuple(route)

    tried = set()
    pivot_candidates = random.sample(linehauls, min(K * 2, len(linehauls)))

    for pivot in pivot_candidates:
        if pivot in tried:
            continue
        tried.add(pivot)

        # 수요 기준 가까운 linehaul 최대 3개 선택
        lh_subset = []
        demand_sum = 0
        for lh in sorted(linehauls, key=lambda x: dist_mat[pivot][x]):
            if lh in lh_subset:
                continue
            if demand_sum + node_demands[lh] > Q:
                break
            lh_subset.append(lh)
            demand_sum += node_demands[lh]

        # backhaul 필터링
        b_subset = [b for b in backhauls if node_demands[b] <= Q - demand_sum]

        routes = labeling_algorithm(lh_subset, b_subset, dist_mat, node_demands, node_types, Q)

        for route, cost in routes:
            route_tuple = tuple(route)
            node_set = frozenset(route[1:-1])

            if route_tuple in unique_route_set:
                continue  # 순서까지 같은 route 중복 제거
            unique_route_set.add(route_tuple)

            # 노드 집합이 같으면 더 나은 cost로 갱신
            if node_set not in best_by_node_set or cost < best_by_node_set[node_set][1]:
                best_by_node_set[node_set] = (route, cost)

        print(f"[DEBUG] Found {len(routes)} routes for pivot {pivot} with lh_subset={lh_subset}")

    route_pool = list(best_by_node_set.values())

    print("[DEBUG] Total routes after filtering:", len(route_pool))
    for r, _ in route_pool:
        print(" - Route:", r)

    if not route_pool:
        print("[ERROR] 초기 route를 단 하나도 생성하지 못했습니다.")

    return route_pool