import random
from typing import List, Dict
import matplotlib.pyplot as plt
from matplotlib import colormaps

def generate_grasp_routes(instance: Dict, alpha: float = 0.3, max_routes: int = None) -> List[List[int]]:
    if max_routes is None:
        max_routes = instance["num_vehicles"]

    num_nodes = instance['num_nodes']
    demands = instance['demands']
    types = instance['node_types']
    capacity = instance['capacity']
    dist = instance['dist_matrix']
    depot = instance['depot_index']

    unvisited = set(range(num_nodes))  # 방문하지 않은 노드 집합
    routes = []  # 결과 경로 리스트

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
                if types[i] == 'linehaul' and demands[i] + curr_load <= capacity
            ]

            if not candidates:
                break

            # 남은 linehaul 개수 확인
            remaining_linehauls = len([i for i in unvisited if types[i] == 'linehaul'])
            remaining_routes = max_routes - len(routes)

            # 앞으로의 route에 linehaul 최소 1개씩 남겨야 함 -> 현재 route에서 멈춤
            if remaining_linehauls < remaining_routes:
                break

            sorted_candidates = sorted(candidates, key=lambda i: dist[current][i])
            alpha_list_size = max(1, int(len(sorted_candidates) * alpha))
            selected = random.choice(sorted_candidates[:alpha_list_size])

            route_linehaul.append(selected)
            curr_load += demands[selected]
            unvisited.remove(selected)
            current = selected

        curr_load -= sum(demands[i] for i in route_linehaul)

        # backhaul 선택
        for i in list(unvisited):
            if types[i] == 'backhaul' and demands[i] + curr_load <= capacity:
                route_backhaul.append(i)
                curr_load += demands[i]
                unvisited.remove(i)

        # linehaul이 하나라도 있어야 valid
        if route_linehaul:
            full_route = [depot] + route_linehaul + route_backhaul + [depot]
            routes.append(full_route)

    return routes

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
                    legend_label_flag = False  # 다음부턴 표시하지 않음
            elif types[i] == 'linehaul':
                plt.scatter(x, y, color=colors(idx), marker='o', s=80)
            elif types[i] == 'backhaul':
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
    from HSH_Instance import generate_instance

    instance = generate_instance(
        num_nodes = 50,
        linehaul_ratio = 0.66,
        capacity = 5000,
        num_vehicles = 6
    )

    routes = generate_grasp_routes(instance, alpha=0.3)

    dist = instance['dist_matrix']

    print("[생성된 경로]")
    total_distance = 0
    for idx, route in enumerate(routes):
        print(f"차량 {idx+1}: {route}")

        dist_sum = sum(dist[route[i]][route[i+1]] for i in range(len(route)-1))
        print(f"  -> 경로 {idx+1} 거리: {dist_sum:.2f}")
        total_distance += dist_sum

    print(f"\n[총 이동 거리] {total_distance:.2f}")
    plot_routes(instance, routes)