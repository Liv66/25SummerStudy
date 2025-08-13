# HSH_TS: Tabu Search
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

def tabu_search(instance: Dict, route_set: List[List[int]], max_iter: int = 50, tabu_tenure: int = 9) -> List[List[int]]:
    from collections import deque
    import copy

    dist = instance["dist_mat"]
    types = instance["node_types"]
    demands = instance["node_demands"]
    capacity = instance["capa"]
    depot = 0

    # 근접 후보 수(Granular Neighborhood)
    nn_m = 12

    N = len(dist)
    nearest = [sorted(range(N), key=lambda j: dist[i][j])[:nn_m] for i in range(N)]

    def rcost(route: List[int]) -> float:
        return sum(dist[route[i]][route[i+1]] for i in range(len(route)-1))

    def pivot_idx(route: List[int]) -> int:
        for t in range(1, len(route)-1):
            if types[route[t]] == 2:
                return t
        return len(route)-1

    def feasible(route: List[int]) -> bool:
        return is_feasible(route, types, demands, capacity, depot)

    # 초기화
    current_solution = copy.deepcopy(route_set)
    best_solution = copy.deepcopy(route_set)
    route_costs = [rcost(r) for r in current_solution]
    best_cost = sum(route_costs)

    tabu_list = deque()
    def is_tabu(attr): return attr in tabu_list
    def push_tabu(attr):
        tabu_list.append(attr)
        if len(tabu_list) > tabu_tenure:
            tabu_list.popleft()

    for _ in range(max_iter):
        pivots = [pivot_idx(r) for r in current_solution]

        best_neighbor = None
        best_neighbor_delta = float('inf')
        best_attr = None

        base_sum = sum(route_costs)

        # Intra 2-opt
        for ri, route in enumerate(current_solution):
            L = len(route)
            if L <= 4:
                continue
            p = pivots[ri]

            for i in range(1, max(1, p-1)):
                for k in range(i+1, p):
                    a, b = route[i-1], route[i]
                    c, d = route[k], route[k+1]
                    delta = (dist[a][c] + dist[b][d]) - (dist[a][b] + dist[c][d])
                    if delta >= best_neighbor_delta:
                        continue
                    new_r = route[:i] + route[i:k+1][::-1] + route[k+1:]
                    if not feasible(new_r):
                        continue
                    attr = ("2optL", ri, i, k)
                    if is_tabu(attr) and (base_sum + delta) >= best_cost:
                        continue
                    nb = current_solution.copy()
                    nb[ri] = new_r
                    best_neighbor = nb
                    best_neighbor_delta = delta
                    best_attr = attr

            for i in range(max(p,1), L-2):
                for k in range(i+1, L-1):
                    a, b = route[i-1], route[i]
                    c, d = route[k], route[k+1]
                    delta = (dist[a][c] + dist[b][d]) - (dist[a][b] + dist[c][d])
                    if delta >= best_neighbor_delta:
                        continue
                    new_r = route[:i] + route[i:k+1][::-1] + route[k+1:]
                    if not feasible(new_r):
                        continue
                    attr = ("2optB", ri, i, k)
                    if is_tabu(attr) and (base_sum + delta) >= best_cost:
                        continue
                    nb = current_solution.copy()
                    nb[ri] = new_r
                    best_neighbor = nb
                    best_neighbor_delta = delta
                    best_attr = attr

        # Or-opt
        for ri, route in enumerate(current_solution):
            L = len(route)
            if L <= 4:
                continue
            p = pivots[ri]
            for blk in (1,2,3):

                for s in range(1, max(1, p-blk)+1):
                    if s+blk > p: break
                    a = route[s-1]; b = route[s]; c = route[s+blk-1]; d = route[s+blk]
                    for ins in range(1, p-(blk-1)):
                        if ins >= s and ins <= s+blk:
                            continue
                        delta_rem = (dist[a][d] - (dist[a][b] + dist[c][d]))
                        u = route[ins-1]; v = route[ins]
                        delta_ins = (dist[u][b] + dist[c][v] - dist[u][v])
                        delta = delta_rem + delta_ins
                        if delta >= best_neighbor_delta:
                            continue
                        new_r = route[:s] + route[s+blk:]
                        new_r = new_r[:ins] + route[s:s+blk] + new_r[ins:]
                        if not feasible(new_r):
                            continue
                        attr = ("orL", ri, s, blk, ins)
                        if is_tabu(attr) and (base_sum + delta) >= best_cost:
                            continue
                        nb = current_solution.copy()
                        nb[ri] = new_r
                        best_neighbor = nb
                        best_neighbor_delta = delta
                        best_attr = attr

                for s in range(max(p,1), L-1-blk+1):
                    if s+blk > L-1: break
                    a = route[s-1]; b = route[s]; c = route[s+blk-1]; d = route[s+blk]
                    for ins in range(max(p,1), L-(blk-1)):
                        if ins >= s and ins <= s+blk:
                            continue
                        delta_rem = (dist[a][d] - (dist[a][b] + dist[c][d]))
                        u = route[ins-1]; v = route[ins]
                        delta_ins = (dist[u][b] + dist[c][v] - dist[u][v])
                        delta = delta_rem + delta_ins
                        if delta >= best_neighbor_delta:
                            continue
                        new_r = route[:s] + route[s+blk:]
                        new_r = new_r[:ins] + route[s:s+blk] + new_r[ins:]
                        if not feasible(new_r):
                            continue
                        attr = ("orB", ri, s, blk, ins)
                        if is_tabu(attr) and (base_sum + delta) >= best_cost:
                            continue
                        nb = current_solution.copy()
                        nb[ri] = new_r
                        best_neighbor = nb
                        best_neighbor_delta = delta
                        best_attr = attr

        # Swap(경로 간 LH <-> LH, BH <-> BH)
        R = len(current_solution)
        for i in range(R):
            ri = current_solution[i]; pi = pivots[i]
            for j in range(i+1, R):
                rj = current_solution[j]; pj = pivots[j]
                # LH <-> LH
                for ai in range(1, pi):
                    u = ri[ai]
                    for aj in range(1, pj):
                        v = rj[aj]
                        # 근접 후보 제한(둘 중 하나라도 가까우면 통과)
                        if (v not in nearest[u]) and (u not in nearest[v]):
                            continue
                        iu0, iu1 = ri[ai-1], ri[ai+1]
                        jv0, jv1 = rj[aj-1], rj[aj+1]
                        delta = 0.0
                        delta += (dist[iu0][v] + dist[v][iu1] - (dist[iu0][u] + dist[u][iu1]))
                        delta += (dist[jv0][u] + dist[u][jv1] - (dist[jv0][v] + dist[v][jv1]))
                        if delta >= best_neighbor_delta:
                            continue
                        new_i = ri[:]; new_i[ai] = v
                        new_j = rj[:]; new_j[aj] = u
                        if not feasible(new_i) or not feasible(new_j):
                            continue
                        attr = ("swapL", i, ai, j, aj)
                        if is_tabu(attr) and (base_sum + delta) >= best_cost:
                            continue
                        nb = current_solution.copy()
                        nb[i] = new_i; nb[j] = new_j
                        best_neighbor = nb
                        best_neighbor_delta = delta
                        best_attr = attr

                # BH <-> BH
                for ai in range(max(pi,1), len(ri)-1):
                    u = ri[ai]
                    if types[u] != 2:
                        continue
                    for aj in range(max(pj,1), len(rj)-1):
                        v = rj[aj]
                        if types[v] != 2:
                            continue
                        if (v not in nearest[u]) and (u not in nearest[v]):
                            continue
                        iu0, iu1 = ri[ai-1], ri[ai+1]
                        jv0, jv1 = rj[aj-1], rj[aj+1]
                        delta = 0.0
                        delta += (dist[iu0][v] + dist[v][iu1] - (dist[iu0][u] + dist[u][iu1]))
                        delta += (dist[jv0][u] + dist[u][jv1] - (dist[jv0][v] + dist[v][jv1]))
                        if delta >= best_neighbor_delta:
                            continue
                        new_i = ri[:]; new_i[ai] = v
                        new_j = rj[:]; new_j[aj] = u
                        if not feasible(new_i) or not feasible(new_j):
                            continue
                        attr = ("swapB", i, ai, j, aj)
                        if is_tabu(attr) and (base_sum + delta) >= best_cost:
                            continue
                        nb = current_solution.copy()
                        nb[i] = new_i; nb[j] = new_j
                        best_neighbor = nb
                        best_neighbor_delta = delta
                        best_attr = attr

        # Relocate
        for i in range(len(current_solution)):
            for j in range(len(current_solution)):
                if i == j:
                    continue
                route_from = current_solution[i]
                route_to = current_solution[j]
                for idx in range(1, len(route_from) - 1):
                    node = route_from[idx]
                    # 근접 후보: node의 near에 있는 정점 주변 위치만 삽입 후보
                    for insert_pos in range(1, len(route_to)):
                        prev = route_to[insert_pos-1]; nxt = route_to[insert_pos]
                        if (prev not in nearest[node]) and (nxt not in nearest[node]):
                            continue
                        new_from = route_from[:idx] + route_from[idx+1:]
                        new_to = route_to[:insert_pos] + [node] + route_to[insert_pos:]
                        if not feasible(new_from) or not feasible(new_to):
                            continue

                        a, b = route_from[idx-1], node
                        c = route_from[idx+1]
                        delta_rem = dist[a][c] - (dist[a][b] + dist[b][c])
                        u, v = route_to[insert_pos-1], route_to[insert_pos]
                        delta_ins = (dist[u][node] + dist[node][v] - dist[u][v])
                        delta = delta_rem + delta_ins
                        if delta >= best_neighbor_delta:
                            continue
                        attr = ("rel", node, i, j, idx, insert_pos)
                        if is_tabu(attr) and (base_sum + delta) >= best_cost:
                            continue
                        nb = current_solution.copy()
                        nb[i] = new_from
                        nb[j] = new_to
                        best_neighbor = nb
                        best_neighbor_delta = delta
                        best_attr = attr

        # 적용 또는 종료
        if best_neighbor is None:
            break

        # 현재 해 갱신
        current_solution = best_neighbor

        route_costs = [rcost(r) for r in current_solution]
        cur_sum = sum(route_costs)

        # Tabu push
        push_tabu(best_attr)

        # Best 갱신
        if cur_sum < best_cost:
            best_cost = cur_sum
            best_solution = [r[:] for r in current_solution]

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

if __name__ == '__main__':
    random.seed(42)
    from HSH_GRASP import generate_grasp_routes
    from HSH_loader import load_instance_from_json

    instance = load_instance_from_json(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_130_0.85.json")
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
    improved_routes = tabu_search(instance, route_set, max_iter=50, tabu_tenure=9)

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
