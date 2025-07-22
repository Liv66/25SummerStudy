from collections import deque

def solve_pricing_problem(duals, node_types, node_demands, capa, dist_mat, forbidden_nodes=None):
    """
    개선된 Labeling 기반 Pricing Problem Solver (첫 개선 route에서 조기 종료)
    """
    print("[DEBUG] Start Pricing Problem with Labeling Algorithm")

    N = len(node_types)
    if forbidden_nodes is None:
        forbidden_nodes = set()
    else:
        print(f"[DEBUG] Forbidden customer nodes: {sorted(forbidden_nodes)}")

    Label = lambda node, visited, demand, cost, path: (node, visited, demand, cost, path)
    queue = deque()
    queue.append(Label(0, set(), 0, 0.0, [0]))

    label_count = 0
    pruned_count = 0
    complete_routes = 0
    visited_routes = set()

    while queue:
        node, visited, demand, cost, path = queue.popleft()
        label_count += 1

        # Backhaul 선입 방지 (단, depot에서 바로 backhaul은 허용)
        if node != 0 and node_types[node] == 2:
            if any(node_types[i] == 1 for i in range(N) if i not in visited and i not in forbidden_nodes):
                pruned_count += 1
                continue

        # 노드 방문 시 처리
        if node != 0:
            if node in forbidden_nodes:
                pruned_count += 1
                continue
            demand += node_demands[node]
            if demand > capa:
                pruned_count += 1
                continue
            visited = visited | {node}

        # depot 복귀 경로 평가
        if path[-1] == 0 and len(path) > 2:
            route_tuple = tuple(path)
            if route_tuple in visited_routes:
                continue
            visited_routes.add(route_tuple)

            complete_routes += 1
            reduced = cost - sum(duals[i - 1] for i in path if i != 0)
            if reduced < 0:
                print(f"[DEBUG] First improving route found: {path} | Reduced cost = {reduced:.4f}")
                print("========== [PRICING DEBUG REPORT] ==========")
                print(f"Total labels generated: {label_count}")
                print(f"Labels pruned (backhaul/overload/forbidden): {pruned_count}")
                print(f"Complete routes evaluated: {complete_routes}")
                print(f"Best route: {path}")
                print(f"Best reduced cost: {reduced:.4f}")
                print("============================================")
                return list(path), reduced
            continue

        # 다음 노드 확장 (dual 기준 정렬)
        customer_nodes = [i for i in range(1, N)]
        next_nodes = sorted(customer_nodes, key=lambda i: -duals[i - 1])
        for next_node in next_nodes:
            if next_node == node or next_node in visited or next_node in forbidden_nodes:
                continue
            queue.append(Label(
                next_node,
                visited.copy(),
                demand,
                cost + dist_mat[node][next_node],
                path + [next_node]
            ))

        # depot 복귀 시도 (최소 2개 이상 방문 후에만 허용)
        if node != 0 and len(visited) >= 2 and path[-1] != 0:
            queue.append(Label(
                0,
                visited.copy(),
                demand,
                cost + dist_mat[node][0],
                path + [0]
            ))

    # 개선 패턴이 없을 경우
    print("========== [PRICING DEBUG REPORT] ==========")
    print(f"Total labels generated: {label_count}")
    print(f"Labels pruned (backhaul/overload/forbidden): {pruned_count}")
    print(f"Complete routes evaluated: {complete_routes}")
    print("No feasible route found with negative reduced cost.")
    print("============================================")
    return [], float("inf")