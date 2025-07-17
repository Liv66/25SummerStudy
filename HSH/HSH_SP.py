from gurobipy import Model, GRB, quicksum
from typing import List, Dict, Tuple
from HSH_Instance import generate_instance
from HSH_GeneratePool import run_pooling_loop
import matplotlib.pyplot as plt
from matplotlib import colormaps
import time

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

    # Unvisited node
    unvisited_nodes = [i for i in range(num_nodes) if i != depot_index and i not in visited_nodes]
    for i in unvisited_nodes:
        x, y = coords[i]
        plt.scatter(x, y, color='gray', marker='x', s=80)
    plt.scatter([], [], color='gray', marker='x', s=80, label='Unvisited')

    plt.title('SP Optimal Routes')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.legend()
    plt.show()

def run_set_partitioning(instance: Dict, pool: List[List[int]]):
    num_nodes = instance["num_nodes"]
    depot = instance["depot_index"]
    dist = instance["dist_matrix"]
    num_vehicles = instance["num_vehicles"]

    customer_indices = set(range(num_nodes))
    customer_indices.discard(depot)

    model = Model("SetPartitioning")
    model.setParam("OutputFlag", 1)

    # Route 변수 생성
    route_vars = []
    for r_idx, route in enumerate(pool):
        var = model.addVar(vtype=GRB.BINARY, name=f"route_{r_idx}")
        route_vars.append(var)
    model.update()

    # 고객 커버 제약: depot 제외한 노드만 검사
    for c in customer_indices:
        model.addConstr(
            quicksum(route_vars[r_idx] for r_idx, route in enumerate(pool)
                     if c in route[1:-1]) == 1,
            name=f"cover_customer_{c}"
        )

    # [제약 2] 선택된 route 수는 차량 수 이하여야 함
    model.addConstr(
        quicksum(route_vars) <= num_vehicles,
        name="vehicle_limit"
    )

    # 목적함수: 총 이동 거리 최소화
    model.setObjective(
        quicksum(
            route_vars[r_idx] * sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))
            for r_idx, route in enumerate(pool)
        ),
        GRB.MINIMIZE
    )

    model.optimize()

    if model.status == GRB.OPTIMAL:
        print("\n[선택된 최적 route 조합]")
        total_cost = 0
        for r_idx, var in enumerate(route_vars):
            if var.X > 0.5:
                route = pool[r_idx]
                cost = sum(dist[route[i]][route[i + 1]] for i in range(len(route) - 1))
                types = instance["node_types"]
                depot = instance["depot_index"]
                typed_route = [(node, 'depot' if node == depot else types[node]) for node in route]
                # typed_route = [(node, types[node]) for node in route]
                print(f"Route {r_idx + 1}: {typed_route} (거리: {cost:.2f})")
                total_cost += cost
        print(f"\n[총 이동 거리(SP 최적화 결과)] {total_cost:.2f}")
        # 시각화용 selected routes 추출
        selected_routes = [pool[r_idx] for r_idx, var in enumerate(route_vars) if var.X > 0.5]
        plot_routes(instance, selected_routes)
    else:
        print("No optimal solution found.")

if __name__ == '__main__':
    start_time = time.time()
    instance = generate_instance(
        num_nodes = 50,
        linehaul_ratio = 0.66,
        capacity = 5000,
        num_vehicles = 6
    )

    pool = run_pooling_loop(instance, duration_seconds = 29)

    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")

    run_set_partitioning(instance, pool)

    end_time = time.time()
    print(f"\n[전체 알고리즘 실행 시간] {end_time - start_time:.2f}초")