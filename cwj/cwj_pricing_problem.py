# cwj_pricing_problem.py
import heapq

def labeling_algorithm(linehauls, backhauls, dist_mat, node_demands, node_types, Q):
    """
    Labeling 알고리즘 (linehaul → backhaul 순서), capa 제약 반영
    """
    depot = 0
    routes = []

    def is_feasible(load):
        return 0 <= load <= Q

    def extend_path(path, visited, load, cost, phase):
        last = path[-1]
        candidates = linehauls if phase == 'L' else backhauls
        for nxt in candidates:
            if nxt in visited:
                continue
            if phase == 'L' and any(node_types[i] == 2 for i in visited):
                continue  # backhaul 방문 후 linehaul 금지

            # load 업데이트
            new_load = load - node_demands[nxt] if node_types[nxt] == 1 else load + node_demands[nxt]
            if not is_feasible(new_load):
                continue

            new_cost = cost + dist_mat[last][nxt]
            new_phase = 'L' if node_types[nxt] == 1 else 'B'
            yield path + [nxt], visited | {nxt}, new_load, new_cost, new_phase

    # 초기 상태: depot에서 가득 찬 상태로 출발
    queue = [(0, [depot], {depot}, Q, 'L')]
    while queue:
        cost, path, visited, load, phase = heapq.heappop(queue)
        last = path[-1]

        if len(path) > 2 and (phase == 'B' or all(node_types[i] == 1 for i in path[1:])):
            full_path = path + [depot]
            full_cost = cost + dist_mat[last][depot]
            routes.append((full_path, full_cost))

        for new_path, new_visited, new_load, new_cost, new_phase in extend_path(path, visited, load, cost, phase):
            heapq.heappush(queue, (new_cost, new_path, new_visited, new_load, new_phase))

    # 추가: 어떤 고객이 커버되지 않았는지 확인
    covered = set()
    for route, _ in routes:
        covered.update(route)
    missed = set(i for i in range(1, len(node_types))) - covered
    if missed:
        print(f"[WARNING] The following customers are not covered by any route: {missed}")

    return routes