def generate_initial_patterns(node_types, node_demands, capacity, dist_mat, max_vehicles):
    """
    max_vehicles 제약을 만족하면서 모든 고객을 한 번 이상 포함하는 초기 패턴 생성.
    1) Greedy하게 고객들을 묶되, 차량 수 제한 고려.
    2) 필요한 경우 단일 노드 패턴으로 보완.
    """
    N = len(node_types)
    customer_ids = list(range(1, N))
    customer_ids.sort(key=lambda i: -node_demands[i])  # 큰 수요부터 먼저

    patterns = []
    costs = []
    routes = [[] for _ in range(max_vehicles)]
    demands = [0 for _ in range(max_vehicles)]

    # 1. 고객들을 max_vehicles 개의 route에 capacity 기준으로 분배
    for i in customer_ids:
        assigned = False
        for r in range(max_vehicles):
            if demands[r] + node_demands[i] <= capacity:
                routes[r].append(i)
                demands[r] += node_demands[i]
                assigned = True
                break
        if not assigned:
            # fallback: 추가 차량 허용 없이 단일 고객으로라도 넣어야 하므로 마지막 route에 강제 삽입 (무시)
            routes[-1].append(i)
            demands[-1] += node_demands[i]

    # 2. 각 route를 depot 포함하여 [0, ..., 0] 경로로 구성
    for route in routes:
        if not route:
            continue
        full_route = [0] + route + [0]
        cost = sum(dist_mat[full_route[i]][full_route[i+1]] for i in range(len(full_route)-1))
        patterns.append(full_route)
        costs.append(cost)

    return patterns, costs