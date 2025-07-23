# cwj_master_postprocess.py

def generate_augmented_routes(route_pool, missed_customers, Q, node_types, node_demands, dist_mat):
    """
    기존 route 중 하나에 missed_customer를 적절한 위치에 삽입하여 새로운 feasible route를 생성
    
    Args:
        route_pool: List of (route, cost) tuples
        missed_customers: Set of customers not covered
        Q: vehicle capacity
        node_types: list of node types (0=depot, 1=linehaul, 2=backhaul)
        node_demands: list of node demands
        dist_mat: distance matrix

    Returns:
        augmented_routes: list of new (route, cost) tuples
    """
    augmented_routes = []
    depot = 0

    for original_route, _ in route_pool:
        current_route = list(original_route)
        remaining = set(missed_customers)
        inserted_any = False

        for customer in list(remaining):
            inserted = False
            for i in range(1, len(current_route)):  # depot 제외 위치들 사이에 삽입
                new_route = current_route[:i] + [customer] + current_route[i:]
                if is_valid_route(new_route, Q, node_types, node_demands):
                    new_cost = compute_route_cost(new_route, dist_mat)
                    current_route = new_route  # 이 route를 기준으로 다음 삽입 시도
                    remaining.remove(customer)
                    inserted = True
                    inserted_any = True
                    # print(f"[POSTPROCESS] ✅ Inserted customer {customer} at position {i}")
                    # print(f"[POSTPROCESS]     → New route: {new_route} | Cost: {new_cost:.2f}")
                    break  # 한 번 삽입되면 다음 customer로 이동
            # if not inserted:
            #     print(f"[POSTPROCESS] ❌ Failed to insert customer {customer} into route {current_route}")

        # 삽입이 1개라도 되었으면 route 추가
        if inserted_any:
            final_cost = compute_route_cost(current_route, dist_mat)
            augmented_routes.append((current_route, final_cost))

        # 이미 다 삽입했다면 더 돌 필요 없음
        if not remaining:
            break

    if remaining:
        for c in remaining:
            print(f"[POSTPROCESS] ⚠️ 최종적으로도 삽입 실패: customer {c}")

    return augmented_routes


def is_valid_route(route, Q, node_types, node_demands):
    """
    linehaul -> backhaul 순서와 용량 제약을 만족하는지 확인
    """
    load = Q
    backhaul_started = False

    for node in route[1:-1]:
        if node_types[node] == 1:  # linehaul
            if backhaul_started:
                return False
            load -= node_demands[node]
        elif node_types[node] == 2:  # backhaul
            backhaul_started = True
            load += node_demands[node]

        if load < 0 or load > Q:
            return False

    return True


def compute_route_cost(route, dist_mat):
    return sum(dist_mat[route[i]][route[i + 1]] for i in range(len(route) - 1))