from copy import deepcopy
from collections import defaultdict

def prune_redundant_customers(routes, node_types, node_demands, dist_mat, Q):
    def route_cost(route):
        return sum(dist_mat[route[i]][route[i+1]] for i in range(len(route)-1))

    # 1. 고객별 방문 횟수 계산
    customer_visit_count = defaultdict(int)
    for route, _ in routes:
        for node in route[1:-1]:
            customer_visit_count[node] += 1

    # 2. 중복 방문 고객만 추출
    redundant_customers = {i for i, c in customer_visit_count.items() if c > 1}

    updated_routes = deepcopy(routes)

    print(f"[PHASE 3] 중복 고객 pruning 시작: 중복 고객 수 = {len(redundant_customers)} → {sorted(redundant_customers)}")

    for cust in redundant_customers:
        candidates = []
        print(f"\n[PHASE 3] → 고객 {cust} 제거 시도:")
        for idx, (route, _) in enumerate(updated_routes):
            if cust in route[1:-1]:
                new_route = route.copy()
                new_route.remove(cust)
                if is_feasible(new_route, node_types, node_demands, Q):
                    new_cost = route_cost(new_route)
                    candidates.append((idx, new_route, new_cost))
                    print(f"   ✔ 제거 가능 (route {idx}) → 새로운 경로: {new_route} | 비용: {new_cost:.0f}")
                else:
                    print(f"   ✘ 제거 불가능 (route {idx}) → 제약 위반")
        if len(candidates) >= 1:
            candidates.sort(key=lambda x: x[2])  # 비용 오름차순
            keep_idx = candidates[-1][0]
            print(f"   → 유지할 경로: route {keep_idx} (비용 {candidates[-1][2]:.0f})")
            for idx, new_r, new_c in candidates[:-1]:
                print(f"     ⤷ route {idx} 에서 고객 {cust} 제거됨 (비용 {new_c:.0f})")
                updated_routes[idx] = (new_r, new_c)

    return updated_routes

def is_feasible(route, node_types, node_demands, Q):
    """
    Linehaul -> Backhaul 순서 및 적재량 제약 확인
    """
    load = Q
    backhaul_started = False
    for i in route[1:-1]:
        if node_types[i] == 2:
            backhaul_started = True
            load += node_demands[i]
        else:
            if backhaul_started:
                return False
            load -= node_demands[i]
        if load < 0 or load > Q:
            return False
    return True