from typing import List, Dict
import matplotlib.pyplot as plt
from matplotlib import colormaps

# 경로 거리 계산
def route_distance(route: List[int], dist: List[List[float]]) -> float:
    return sum(dist[route[i]][route[i+1]] for i in range(len(route)-1))

# 경로 유효성 검사
def is_feasible(route: List[int], types: List[str], demands: List[int], capacity: int, depot: int) -> bool:
    if route[0] != depot or route[-1] != depot:
        return False
    visited = set()
    total_demand = 0
    has_linehaul = False
    after_backhaul = False
    for node in route[1:-1]:
        if node in visited:
            return False
        visited.add(node)
        total_demand += demands[node]
        if total_demand > capacity:
            return False
        if types[node] == 'linehaul':
            if after_backhaul:
                return False
            has_linehaul = True
        elif types[node] == 'backhaul':
            after_backhaul = True
    return has_linehaul

# linehaul은 backhaul 앞까지만 삽입 허용
def try_insert_linehaul(route, node, types, demands, capacity, depot, dist):
    best_insertion = None
    best_cost = float('inf')

    # depot 제외하고 backhaul 시작 위치 탐색
    backhaul_start = next((i for i, n in enumerate(route) if n != depot and types[n] == 'backhaul'), len(route))

    for i in range(1, backhaul_start):  # depot 제외, backhaul 전까지
        new_route = route[:i] + [node] + route[i:]
        if is_feasible(new_route, types, demands, capacity, depot):
            cost = route_distance(new_route, dist)
            if cost < best_cost:
                best_cost = cost
                best_insertion = new_route
    return best_insertion

# 남은 노드로 route 생성
def create_routes_for_remaining(nodes, types, demands, capacity, depot):
    new_routes = []
    used = set()
    for i in range(len(nodes)):
        if nodes[i] in used:
            continue
        route = [nodes[i]]
        curr_demand = demands[nodes[i]]
        used.add(nodes[i])
        for j in range(i+1, len(nodes)):
            if nodes[j] not in used and curr_demand + demands[nodes[j]] <= capacity:
                route.append(nodes[j])
                curr_demand += demands[nodes[j]]
                used.add(nodes[j])
        new_routes.append([depot] + route + [depot])
    return new_routes

# 향상된 Local Search
def enhanced_local_search(instance: Dict, route_set: List[List[int]]) -> List[List[int]]:
    dist = instance["dist_matrix"]
    types = instance["node_types"]
    demands = instance["demands"]
    capacity = instance["capacity"]
    depot = instance["depot_index"]
    num_nodes = instance["num_nodes"]

    # Step 1: 2-opt
    improved_routes = []
    for route in route_set:
        best_route = route
        best_cost = route_distance(route, dist)
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                if route[i] == depot or route[j] == depot:
                    continue
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                if is_feasible(new_route, types, demands, capacity, depot):
                    new_cost = route_distance(new_route, dist)
                    if new_cost < best_cost:
                        best_route = new_route
                        best_cost = new_cost
        improved_routes.append(best_route)

    # Step 2: 미방문 노드 삽입
    visited = set()
    for route in improved_routes:
        visited.update(route)
    unvisited = [i for i in range(num_nodes) if i != depot and i not in visited]
    unvisited_linehaul = [i for i in unvisited if types[i] == 'linehaul']
    unvisited_backhaul = [i for i in unvisited if types[i] == 'backhaul']
    leftover_nodes = []

    # 2-1: backhaul → 뒤에서부터 삽입
    for node in unvisited_backhaul:
        inserted = False
        for r_idx, route in enumerate(improved_routes):
            for pos in range(len(route)-1, 0, -1):
                new_route = route[:pos] + [node] + route[pos:]
                if is_feasible(new_route, types, demands, capacity, depot):
                    improved_routes[r_idx] = new_route
                    inserted = True
                    break
            if inserted:
                break
        if not inserted:
            leftover_nodes.append(node)

    # 2-2: linehaul → backhaul 전까지만 삽입
    for node in unvisited_linehaul:
        inserted = False
        for r_idx, route in enumerate(improved_routes):
            new_route = try_insert_linehaul(route, node, types, demands, capacity, depot, dist)
            if new_route:
                improved_routes[r_idx] = new_route
                inserted = True
                break
        if not inserted:
            leftover_nodes.append(node)

    # Step 3: 남은 노드로 새 route 생성 (단, 차량 수 제한 내에서만 추가)
    remaining_vehicle_slots = instance["num_vehicles"] - len(improved_routes)
    if remaining_vehicle_slots > 0:
        new_routes = create_routes_for_remaining(leftover_nodes, types, demands, capacity, depot)
        # 최대 remaining_vehicle_slots 개까지만 추가
        improved_routes.extend(new_routes[:remaining_vehicle_slots])

    # 차량 수 초과 방지
    return improved_routes[:instance["num_vehicles"]]

# 시각화
def plot_routes(instance: Dict, routes: List[List[int]]):
    coords = instance['node_coords'] + [instance['depot_coord']]
    types = instance['node_types']
    depot_index = instance['depot_index']
    num_nodes = instance['num_nodes']
    plt.figure(figsize=(8, 10))
    colors = colormaps['tab20'].resampled(len(routes))

    visited_nodes = set()
    for route in routes:
        visited_nodes.update(route)

    label_depot_shown = False
    for idx, route in enumerate(routes):
        route_coords = [coords[i] for i in route]
        xs, ys = zip(*route_coords)
        plt.plot(xs, ys, color=colors(idx), label=f'Route {idx + 1}')
        for i in route:
            x, y = coords[i]
            if i == depot_index:
                if not label_depot_shown:
                    plt.scatter(x, y, color='black', marker='s', s=200, label='Depot')
                    label_depot_shown = True
                else:
                    plt.scatter(x, y, color='black', marker='s', s=200)
            elif types[i] == 'linehaul':
                plt.scatter(x, y, color=colors(idx), marker='o', s=80)
            elif types[i] == 'backhaul':
                plt.scatter(x, y, color=colors(idx), marker='^', s=80)

    # Unvisited 노드 시각화 및 범례
    unvisited_nodes = [i for i in range(num_nodes) if i != depot_index and i not in visited_nodes]
    for i in unvisited_nodes:
        x, y = coords[i]
        plt.scatter(x, y, color='gray', marker='x', s=80)
    plt.scatter([], [], color='gray', marker='x', s=80, label='Unvisited')

    plt.title('Enhanced Local Search Routes')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.legend()
    plt.show()

# 실행부
if __name__ == '__main__':
    from HSH_GRASP2 import generate_grasp_routes
    from HSH_Instance import generate_instance

    instance = generate_instance(
        num_nodes=50,
        linehaul_ratio=0.66,
        capacity=5000,
        num_vehicles=6
    )

    # GRASP 초기 경로
    route_set = generate_grasp_routes(instance, alpha=0.3)
    dist = instance['dist_matrix']
    print("[GRASP로 생성된 초기 경로]")
    total_grasp_distance = 0
    for idx, route in enumerate(route_set):
        print(f"차량 {idx+1}: {route}")
        d = route_distance(route, dist)
        print(f"  -> 경로 {idx+1} 거리: {d:.2f}")
        total_grasp_distance += d
    print(f"[총 이동 거리 (GRASP)] {total_grasp_distance:.2f}\n")

    visited_grasp = set()
    for route in route_set:
        visited_grasp.update(route)
    unvisited_grasp = [i for i in range(instance["num_nodes"]) if i != instance["depot_index"] and i not in visited_grasp]
    print(f"[GRASP 기준 unvisited node 수] {len(unvisited_grasp)}\n")

    unvisited_linehaul = [i for i in unvisited_grasp if instance["node_types"][i] == "linehaul"]
    unvisited_backhaul = [i for i in unvisited_grasp if instance["node_types"][i] == "backhaul"]

    print(f"  - linehaul: {len(unvisited_linehaul)}개")
    print(f"  - backhaul: {len(unvisited_backhaul)}개\n")

    # Local Search
    improved_routes = enhanced_local_search(instance, route_set)
    print("[Local Search 이후 개선된 경로]")
    total_local_distance = 0
    for idx, route in enumerate(improved_routes):
        print(f"차량 {idx+1}: {route}")
        d = route_distance(route, dist)
        print(f"  -> 경로 {idx+1} 거리: {d:.2f}")
        type_list = [instance["node_types"][i] if i != instance["depot_index"] else "depot" for i in route]
        print(f"  -> 노드 유형: {type_list}")
        total_local_distance += d
    print(f"[총 이동 거리 (Local Search)] {total_local_distance:.2f}")

    visited_local = set()
    for route in improved_routes:
        visited_local.update(route)
    unvisited_local = [i for i in range(instance["num_nodes"]) if i != instance["depot_index"] and i not in visited_local]
    print(f"[Local Search 기준 unvisited node 수] {len(unvisited_local)}")

    unvisited_linehaul = [i for i in unvisited_local if instance["node_types"][i] == "linehaul"]
    unvisited_backhaul = [i for i in unvisited_local if instance["node_types"][i] == "backhaul"]

    print(f"  - linehaul: {len(unvisited_linehaul)}개")
    print(f"  - backhaul: {len(unvisited_backhaul)}개")

    plot_routes(instance, improved_routes)
