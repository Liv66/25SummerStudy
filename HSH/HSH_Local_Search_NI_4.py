from typing import List, Dict, Tuple
import matplotlib.pyplot as plt
from matplotlib import colormaps
import random

# 경로 거리 계산
def route_distance(route: List[int], dist: List[List[float]]) -> float:
    return sum(dist[route[i]][route[i+1]] for i in range(len(route)-1))

# 경로 유효성 검사
def is_feasible(route: List[int], types: List[int], demands: List[int], capacity: int, depot: int) -> bool:
    if route[0] != depot or route[-1] != depot:
        return False

    visited = set()
    after_backhaul = False

    linehaul_nodes = [node for node in route[1:-1] if types[node] == 1]

    # linehaul 수요 합이 capacity 초과면 infeasible
    total_linehaul_demand = sum(demands[node] for node in linehaul_nodes)
    if total_linehaul_demand > capacity:
        return False

    load = total_linehaul_demand

    for node in route[1:-1]:
        if node in visited:
            return False
        visited.add(node)

        if types[node] == 1:
            if after_backhaul:
                return False  # backhaul 이후에 linehaul이 나오면 오류
            load -= demands[node]
            if load < 0:
                return False
        elif types[node] == 2:
            if not after_backhaul:
                after_backhaul = True
                load = 0
            load += demands[node]
            if load > capacity:
                return False

    return True

def check_capacity_violation(instance: Dict, routes: List[List[int]]):
    print("\n[용량 초과 여부 점검]")

    types = instance["node_types"]
    demands = instance["node_demands"]
    capacity = instance["capa"]
    depot = 0

    for idx, route in enumerate(routes):
        load_seq = []
        visited = set()
        after_backhaul = False

        # depot 출발 시 linehaul 총 수요량을 실음
        linehaul_nodes = [node for node in route[1:-1] if types[node] == 1]
        load = sum(demands[node] for node in linehaul_nodes)

        for node in route:
            if node == depot:
                load_seq.append(load)
                continue
            if node in visited:
                continue
            visited.add(node)

            if types[node] == 1:
                if after_backhaul:
                    load = float('inf') # 순서 오류
                else:
                    load -= demands[node]
            elif types[node] == 2:
                if not after_backhaul:
                    after_backhaul = True
                    load = 0
                load += demands[node]

            load_seq.append(load)

        feasible = all(0 <= l <= capacity for l in load_seq)
        status = "Y" if feasible else "N"
        print(f"차량 {idx + 1} 적재량 변화: {load_seq} → {status}")

def tabu_search(instance: Dict, route_set: List[List[int]], max_iter: int = 50, tabu_tenure: int = 7) -> List[List[int]]:
    from collections import deque
    import copy

    dist = instance["dist_mat"]
    types = instance["node_types"]
    demands = instance["node_demands"]
    capacity = instance["capa"]
    depot = 0

    best_solution = copy.deepcopy(route_set)
    best_cost = sum(route_distance(route, dist) for route in best_solution)

    current_solution = copy.deepcopy(route_set)
    tabu_list = deque()

    for _ in range(max_iter):
        neighborhood = []
        move_candidates = []

        # Generate neighbors via relocation
        for i in range(len(current_solution)):
            for j in range(len(current_solution)):
                if i == j:
                    continue
                route_from = current_solution[i]
                route_to = current_solution[j]
                for idx in range(1, len(route_from) - 1):
                    node = route_from[idx]
                    new_from = route_from[:idx] + route_from[idx+1:]
                    for insert_pos in range(1, len(route_to)):
                        new_to = route_to[:insert_pos] + [node] + route_to[insert_pos:]
                        if is_feasible(new_from, types, demands, capacity, depot) and is_feasible(new_to, types, demands, capacity, depot):
                            new_solution = copy.deepcopy(current_solution)
                            new_solution[i] = new_from
                            new_solution[j] = new_to
                            move = (node, i, j)
                            if move not in tabu_list:
                                cost = sum(route_distance(r, dist) for r in new_solution)
                                neighborhood.append((cost, new_solution, move))

        if not neighborhood:
            break

        neighborhood.sort()
        best_neighbor_cost, best_neighbor_sol, best_move = neighborhood[0]

        current_solution = best_neighbor_sol
        tabu_list.append(best_move)
        if len(tabu_list) > tabu_tenure:
            tabu_list.popleft()

        if best_neighbor_cost < best_cost:
            best_solution = best_neighbor_sol
            best_cost = best_neighbor_cost

    return best_solution

# 시각화
def plot_routes(instance: Dict, routes: List[List[int]]):
    coords = instance['node_coords']
    types = instance['node_types']
    depot_index = 0
    num_nodes = instance['N']
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
            elif types[i] == 1:
                plt.scatter(x, y, color=colors(idx), marker='o', s=80)
            elif types[i] == 2:
                plt.scatter(x, y, color=colors(idx), marker='^', s=80)

    unvisited_nodes = [i for i in range(num_nodes) if i != depot_index and i not in visited_nodes]
    for i in unvisited_nodes:
        x, y = coords[i]
        plt.scatter(x, y, color='gray', marker='x', s=80)
    plt.scatter([], [], color='gray', marker='x', s=80, label='Unvisited')

    plt.title('Tabu Search Routes')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.legend()
    plt.show()

# 실행부
if __name__ == '__main__':
    random.seed(42)
    from HSH_GRASP_NI import generate_grasp_routes
    from HSH_loader import load_instance_from_json

    instance = load_instance_from_json(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_20_0.7.json")
    instance["node_demands"] = [abs(d) for d in instance["node_demands"]]
    route_set = generate_grasp_routes(instance, alpha=0.3)
    dist = instance['dist_mat']
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
    unvisited_grasp = [i for i in range(instance["N"]) if i != 0 and i not in visited_grasp]
    print(f"[GRASP 기준 unvisited node 수] {len(unvisited_grasp)}\n")

    unvisited_linehaul = [i for i in unvisited_grasp if instance["node_types"][i] == 1]
    unvisited_backhaul = [i for i in unvisited_grasp if instance["node_types"][i] == 2]

    print(f"  - linehaul: {len(unvisited_linehaul)}개")
    print(f"  - backhaul: {len(unvisited_backhaul)}개\n")

    route_set = generate_grasp_routes(instance, alpha=0.3)
    # max_iter = 50
    improved_routes = tabu_search(instance, route_set, max_iter=50, tabu_tenure=7)

    print("[Local Search 이후 개선된 경로]")
    total_local_distance = 0
    for idx, route in enumerate(improved_routes):
        print(f"차량 {idx+1}: {route}")
        d = route_distance(route, dist)
        print(f"  -> 경로 {idx+1} 거리: {d:.2f}")
        type_list = [0 if i == 0 else instance["node_types"][i] for i in route]
        print(f"  -> 노드 유형: {type_list}")
        total_local_distance += d
    print(f"[총 이동 거리 (Local Search)] {total_local_distance:.2f}")

    visited_local = set()
    for route in improved_routes:
        visited_local.update(route)
    unvisited_local = [i for i in range(instance["N"]) if i != 0 and i not in visited_local]
    print(f"[Local Search 기준 unvisited node 수] {len(unvisited_local)}")

    unvisited_linehaul = [i for i in unvisited_local if instance["node_types"][i] == 1]
    unvisited_backhaul = [i for i in unvisited_local if instance["node_types"][i] == 2]

    print(f"  - linehaul: {len(unvisited_linehaul)}개")
    print(f"  - backhaul: {len(unvisited_backhaul)}개")

    plot_routes(instance, improved_routes)
    check_capacity_violation(instance, improved_routes)

    print("\n[전체 경로 리스트]")
    print("[", end="")
    for i, route in enumerate(improved_routes):
        print(route, end=",\n" if i < len(improved_routes) - 1 else "")
    print("]")

