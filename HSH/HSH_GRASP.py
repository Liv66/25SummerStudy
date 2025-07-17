# Greedy Randomized Adaptive Search Procedure(GRASP), 초기 경로 생성
# depot에서 시작/종료, linehaul->backhaul 순서 및 용량 제약 만족, linehaul 1개 이상 포함, 재방문 불가
# 상위 alpha 후보 중 무작위 선택 -> 여러 차량에 대해 여러 경로 생성

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

    unvisited = set(range(num_nodes)) # 방문하지 않은 노드 집합
    routes = [] # 결과 경로 리스트

    while unvisited and len(routes) < max_routes:
        route = [depot] # depot에서 출발
        current = depot # 현재 위치
        curr_demand = 0 # 누적 수요
        route_linehaul = [] # linehaul 집합
        route_backhaul = [] # backhaul 집합

        # linehaul 선택
        while True:
            candidates = [
                i for i in unvisited
                if types[i] == 'linehaul' and demands[i] + curr_demand <= capacity
            ]
            if not candidates:
                break

            # 거리 기준 정렬
            sorted_candidates = sorted(candidates, key=lambda i: dist[current][i])

            # 상위 alpha 후보 list
            alpha_list_size = max(1, int(len(sorted_candidates) * alpha))
            selected = random.choice(sorted_candidates[:alpha_list_size])

            route_linehaul.append(selected)
            curr_demand += demands[selected]
            unvisited.remove(selected)
            current = selected # 현재 위치 update

        # backhaul 선택
        for i in list(unvisited):
            if types[i] == 'backhaul' and demands[i] + curr_demand <= capacity:
                route_backhaul.append(i)
                curr_demand += demands[i]
                unvisited.remove(i)

        # 경로 유효성 check
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

    # 방문된 노드 추적
    visited_nodes = set()
    for route in routes:
        visited_nodes.update(route)

    for idx, route in enumerate(routes):
        route_coords = [coords[i] for i in route]
        xs, ys = zip(*route_coords)

        plt.plot(xs, ys, color=colors(idx), label=f'Route {idx + 1}')

        for i in route:
            x, y = coords[i]
            if i == depot_index:
                plt.scatter(x, y, color='black', marker='s', s=200, label='Depot' if idx == 0 else "")
            elif types[i] == 'linehaul':
                plt.scatter(x, y, color=colors(idx), marker='o', s=80)
            elif types[i] == 'backhaul':
                plt.scatter(x, y, color=colors(idx), marker='^', s=80)

    # 방문하지 않은 노드 시각화
    unvisited_nodes = [i for i in range(num_nodes) if i != depot_index and i not in visited_nodes]
    for i in unvisited_nodes:
        x, y = coords[i]
        plt.scatter(x, y, color='gray', marker='x', s=80, label='Unvisited' if i == unvisited_nodes[0] else "")

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

        # 경로별 이동 거리
        dist_sum = sum(dist[route[i]][route[i+1]] for i in range(len(route)-1))
        print(f"  -> 경로 {idx+1} 거리: {dist_sum:.2f}")
        total_distance += dist_sum

    print(f"\n[총 이동 거리] {total_distance:.2f}")

    plot_routes(instance, routes)