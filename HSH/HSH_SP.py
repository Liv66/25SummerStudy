# Set Partitioning(SP)
from gurobipy import Model, GRB, quicksum
from typing import List, Dict
from HSH.HSH_GeneratePool import run_pooling_loop
import matplotlib.pyplot as plt
from matplotlib import colormaps
import time, random, json

def plot_routes(problem_info: Dict, node_type, N, routes: List[List[int]]):
    coords = problem_info['node_coords']
    depot = 0
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
            if i == depot:
                if not legend_label_flag:
                    plt.scatter(x, y, color='black', marker='s', s=200, label='Depot')
                    legend_label_flag = False
            elif node_type[i] == 1:
                plt.scatter(x, y, color=colors(idx), marker='o', s=80)
            elif node_type[i] == 2:
                plt.scatter(x, y, color=colors(idx), marker='^', s=80)

    # 미방문 노드
    unvisited_nodes = [i for i in range(N) if i != depot and i not in visited_nodes]
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

def run_set_partitioning(N, K, dist_mat, pool: List[List[int]]):
    depot = 0

    customer_indices = set(range(N))
    customer_indices.discard(depot)

    model = Model("SetPartitioning")
    model.setParam("OutputFlag", 0)

    # route 변수 생성
    route_vars = []
    for r_idx, route in enumerate(pool):
        var = model.addVar(vtype=GRB.BINARY, name=f"route_{r_idx}")
        route_vars.append(var)
    model.update()

    # depot 제외한 노드만 검사
    for c in customer_indices:
        model.addConstr(
            quicksum(route_vars[r_idx] for r_idx, route in enumerate(pool)
                     if c in route[1:-1]) == 1,
            name=f"cover_customer_{c}"
        )

    # 선택된 route 수는 차량 수 이하여야 함
    model.addConstr(
        quicksum(route_vars) <= K,
        name="vehicle_limit"
    )

    # 목적함수: 총 이동 거리 최소화
    model.setObjective(
        quicksum(
            route_vars[r_idx] * sum(dist_mat[route[i]][route[i + 1]] for i in range(len(route) - 1))
            for r_idx, route in enumerate(pool)
        ),
        GRB.MINIMIZE
    )

    model.optimize()

    if model.status == GRB.OPTIMAL:
        # print("\n[선택된 최적 route 조합]")
        selected_routes = []
        total_cost = 0
        for r_idx, var in enumerate(route_vars):
            if var.X > 0.5:
                route = pool[r_idx]
                selected_routes.append(route)

                # node_types = [0 if node == depot else (1 if node_type[node] == 1 else 2) for node in route]
                cost = sum(dist_mat[route[i]][route[i + 1]] for i in range(len(route) - 1))
                # print(f"Route {len(selected_routes)}: {route}")
                # print(f"  -> node_types: {node_types}")
                # print(f"  -> 거리: {cost:.2f}\n")
                total_cost += cost

        # plot_routes(instance, selected_routes)

        # print("\n[전체 경로 리스트]")
        print("[", end="")
        for i, route in enumerate(selected_routes):
            print(route, end=",\n" if i < len(selected_routes) - 1 else "")
        print("]")
        return selected_routes

    else:
        print("No optimal solution found.")

def validate_solution(N, K, node_type, node_demand, capa, selected_routes: List[List[int]]):
    depot = 0

    # 차량 수 확인
    num_used_vehicles = len(selected_routes)
    cond1 = num_used_vehicles <= K
    print(f"1. 사용한 차량 수: {num_used_vehicles} / K = {K} → {cond1}")

    # 각 경로의 node node_type
    all_node_types = []
    cond3_list = []
    for idx, route in enumerate(selected_routes):
        type_seq = [0 if node == depot else node_type[node] for node in route]
        all_node_types.append(type_seq)
        print(f"2. 차량 {idx+1} node node_type: {type_seq}")

        # depot으로 시작/끝, 1 포함, 1-2-1 없는지
        has_linehaul = 1 in type_seq
        is_depot_ends = (type_seq[0] == 0 and type_seq[-1] == 0)
        no_121 = all(not (type_seq[i] == 1 and type_seq[i+1] == 2 and type_seq[i+2] == 1)
                     for i in range(len(type_seq) - 2))
        cond3 = has_linehaul and is_depot_ends and no_121
        cond3_list.append(cond3)
    print(f"3. 모든 경로가 depot 시작/종료 + 1 포함 + 1-2-1 없음 → {all(cond3_list)}")

    # 중복 방문 확인(depot 제외)
    all_visited = []
    for route in selected_routes:
        all_visited.extend(route[1:-1])
    cond4 = len(all_visited) == len(set(all_visited))
    print(f"4. depot 제외 중복 방문 없음 → {cond4}")

    # 미방문 노드 확인
    all_customer = set(range(N)) - {depot}
    cond5 = set(all_visited) == all_customer
    print(f"5. 미방문 노드 없음 → {cond5}")

    # 용량 초과 여부 확인
    cond6_list = []
    for idx, route in enumerate(selected_routes):
        load_seq = []
        visited = set()
        load = 0
        after_backhaul = False

        # depot에서 출발할 때 실어야 할 총 linehaul 수요 계산
        linehaul_nodes = [node for node in route[1:-1] if node_type[node] == 1]
        load = sum(node_demand[node] for node in linehaul_nodes)

        for node in route:
            if node == depot:
                load_seq.append(load)
                continue
            if node in visited:
                continue
            visited.add(node)

            if node_type[node] == 1:
                if after_backhaul:
                    load = float('inf')
                else:
                    load -= node_demand[node]
            elif node_type[node] == 2:
                if not after_backhaul:
                    after_backhaul = True
                    load = 0
                load += node_demand[node]

            load_seq.append(load)

        feasible = all(0 <= l <= capa for l in load_seq)
        cond6_list.append(feasible)
        print(f"6. 차량 {idx + 1} 적재량 변화: {load_seq} → {'Y' if feasible else 'N'}")
    print(f"6. 모든 차량에서 용량 초과 없음 → {all(cond6_list)}")

if __name__ == '__main__':
    random.seed(42)
    with open(r"C:\Users\seohyun\Desktop\25SummerStudy\instances\problem_130_0.85.json", encoding="utf-8") as f:
        problem_info = json.load(f)
    N = problem_info["N"]
    K = problem_info["K"]
    node_type = problem_info['node_types']
    node_demand = [abs(d) for d in problem_info["node_demands"]]
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    start_time = time.time()

    pool = run_pooling_loop(N, K, node_type, node_demand, capa, dist_mat)

    print(f"\n[총 수집된 고유 route 개수] {len(pool)}개")

    selected_routes = run_set_partitioning(N, K, dist_mat, pool)
    end_time = time.time()

    print(f"\n[전체 알고리즘 실행 시간] {end_time - start_time:.2f}초")

    if selected_routes:
        validate_solution(N, K, node_type, node_demand, capa, selected_routes)