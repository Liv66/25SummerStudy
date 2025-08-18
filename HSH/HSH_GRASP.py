# GRASP
import random
from typing import List, Dict
import matplotlib.pyplot as plt
from matplotlib import colormaps
import json

def generate_grasp_routes(N, K, node_type, node_demand, capa, dist_mat) -> List[List[int]]:
    max_routes = K
    depot = 0
    alpha = float(0.3)

    unvisited = set(range(1, N))  # depot 제외
    routes = []

    while unvisited and len(routes) < max_routes:
        route_linehaul = []
        route_backhaul = []
        curr_load = 0
        current = depot

        # linehaul 선택
        while True:
            # 후보 linehaul 노드
            candidates = [
                i for i in unvisited
                if node_type[i] == 1 and node_demand[i] + curr_load <= capa
            ]

            if not candidates:
                break

            # 남은 linehaul 개수 확인
            remaining_linehauls = len([i for i in unvisited if node_type[i] == 1])
            remaining_routes = max_routes - len(routes)

            # 앞으로의 route에 linehaul 최소 1개씩 남겨야 함 -> 현재 route에서 멈춤
            if remaining_linehauls < remaining_routes:
                break

            sorted_candidates = sorted(candidates, key=lambda i: dist_mat[current][i])
            alpha_list_size = max(1, int(len(sorted_candidates) * alpha))
            selected = random.choice(sorted_candidates[:alpha_list_size])

            route_linehaul.append(selected)
            curr_load += node_demand[selected]
            unvisited.remove(selected)
            current = selected

        curr_load -= sum(node_demand[i] for i in route_linehaul)

        # backhaul 선택
        for i in list(unvisited):
            if node_type[i] == 2 and node_demand[i] + curr_load <= capa:
                route_backhaul.append(i)
                curr_load += node_demand[i]
                unvisited.remove(i)

        # linehaul이 하나라도 있어야 valid
        if route_linehaul:
            full_route = [depot] + route_linehaul + route_backhaul + [depot]
            routes.append(full_route)

    return routes

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

    legend_label_flag = True

    for idx, route in enumerate(routes):
        route_coords = [coords[i] for i in route]
        xs, ys = zip(*route_coords)

        plt.plot(xs, ys, color=colors(idx), label=f'Route {idx + 1}')

        for i in route:
            x, y = coords[i]
            if i == depot_index:
                if legend_label_flag:
                    plt.scatter(x, y, color='black', marker='s', s=200, label='Depot')
                    legend_label_flag = False
            elif types[i] == 1:
                plt.scatter(x, y, color=colors(idx), marker='o', s=80)
            elif types[i] == 2:
                plt.scatter(x, y, color=colors(idx), marker='^', s=80)

    # 방문하지 않은 노드 시각화
    unvisited_nodes = [i for i in range(num_nodes) if i != depot_index and i not in visited_nodes]
    for i in unvisited_nodes:
        x, y = coords[i]
        plt.scatter(x, y, color='gray', marker='x', s=80)
    plt.scatter([], [], color='gray', marker='x', s=80, label='Unvisited')

    plt.title('GRASP Routes (with Unvisited Nodes)')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    random.seed(42)
    with open(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_130_0.85.json", encoding="utf-8") as f:
        problem_info = json.load(f)
    N = problem_info['N']
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = [abs(d) for d in problem_info["node_demands"]]
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']
    routes = generate_grasp_routes(N, K, node_type, node_demand, capa, dist_mat)

    print("[생성된 경로]")
    total_distance = 0
    for idx, route in enumerate(routes):
        print(f"차량 {idx+1}: {route}")

        # 노드 타입 리스트
        node_types = [node_type[node] for node in route]
        print(f"  -> node_types: {node_types}")

        dist_sum = sum(dist_mat[route[i]][route[i+1]] for i in range(len(route)-1))
        print(f"  -> 경로 {idx+1} 거리: {dist_sum:.2f}")
        total_distance += dist_sum

    print(f"\n[총 이동 거리] {total_distance:.2f}")
    plot_routes(problem_info, routes)

    print("\n[전체 경로 리스트]")
    print("[", end="")
    for i, route in enumerate(routes):
        print(route, end=",\n" if i < len(routes) - 1 else "")
    print("]")
