import random
import matplotlib.pyplot as plt
import gurobipy as gp
import time
from gurobipy import GRB
import numpy as np
from scipy.stats import gaussian_kde
from scipy.optimize import linear_sum_assignment
import json
import matplotlib
from sympy.codegen.ast import continue_


def two_opt_wb(depot, nodes, dist_matrix, nodes_coord=None, show=False):
    """
    2-OPT 로컬 서치 알고리즘
    depot: int, 시작 및 종료 노드 (보통 0)
    nodes: list[int], depot을 제외한 고객 노드들
    dist_matrix: 2차원 거리 행렬
    nodes_coord: (선택) 시각화용 노드 좌표
    show: bool, True면 경로와 비용 출력 및 시각화
    """
    # 초기 경로: depot → 고객들 → depot
    result_route = [depot] + nodes + [depot]

    # 초기 비용 계산 (전체 경로)
    result_cost = sum(
        dist_matrix[result_route[i]][result_route[i + 1]]
        for i in range(len(result_route) - 1)
    )

    improved = True
    st = time.time()

    while improved:
        improved = False
        # 내부 고객 구간만 2-OPT 대상 (depot은 고정)
        for i in range(1, len(result_route) - 2):
            for j in range(i + 1, len(result_route) - 1):
                # 기존 거리
                before = dist_matrix[result_route[i - 1]][result_route[i]] + \
                         dist_matrix[result_route[j]][result_route[j + 1]]

                # 2-OPT 적용 후 거리
                after = dist_matrix[result_route[i - 1]][result_route[j]] + \
                        dist_matrix[result_route[i]][result_route[j + 1]]

                # 개선되면 경로 반전
                if after < before:
                    result_route = (
                            result_route[:i] +
                            list(reversed(result_route[i:j + 1])) +
                            result_route[j + 1:]
                    )
                    result_cost += after - before
                    improved = True

    if show:
        print("✅ 2-OPT 최적화 결과")
        print(f"총 비용: {result_cost}")
        print(f"경로: {result_route}")
        print(f"소요 시간: {time.time() - st:.4f}초")
        if nodes_coord:
            plot(nodes_coord, result_route, f'2-OPT Cost: {int(result_cost)}')

    return result_cost, result_route


def kde_out(n, m, vehicle_capacity, demands, node_types, coords_dict, dist_matrix):
    capacities = [vehicle_capacity] * m
    depot_idx = 0
    depot_coord = coords_dict[0]
    linehaul_ids = [i for i, t in enumerate(node_types) if t == 1]
    linehaul_coord = [coords_dict[i] for i in linehaul_ids]
    backhaul_ids = [i for i, t in enumerate(node_types) if t == 2]
    backhaul_coord = [coords_dict[i] for i in backhaul_ids]

    # ---- Customer 분포 주청 ----
    kde_linehaul = gaussian_kde(np.array(linehaul_coord).T)
    mins_linehaul = np.min(linehaul_coord, axis=0)
    maxs_linehaul = np.max(linehaul_coord, axis=0)
    kde_backhaul = gaussian_kde(np.array(backhaul_coord).T)
    mins_backhaul = np.min(backhaul_coord, axis=0)
    maxs_backhaul = np.max(backhaul_coord, axis=0)
    return capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul, kde_backhaul, mins_backhaul, maxs_backhaul


def run_sscflp_vrpb(time_left, capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord, backhaul_ids, backhaul_coord,
                    kde_linehaul, mins_linehaul, maxs_linehaul, kde_backhaul, mins_backhaul, maxs_backhaul, n, m,
                    vehicle_capacity, demands, node_types, coords_dict, dist_matrix):
    def euclidean(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def SSCFLP(time_left, customer_coords, customer_idx, kde, mins, maxs):
        nn = len(customer_coords)
        raw_samples = kde.resample(m).T
        clipped_samples = np.clip(raw_samples, mins, maxs)
        facility_coords = [tuple(coord) for coord in clipped_samples]

        transport_cost = [[euclidean(customer_coords[i], facility_coords[j]) for j in range(m)] for i in range(nn)]

        time_limit = max(0.1, time_left)
        model = gp.Model("SSCFLP")
        model.setParam("OutputFlag", 0)
        model.setParam("TimeLimit", time_limit)

        x = model.addVars(nn, m, vtype=GRB.BINARY)
        y = model.addVars(m, vtype=GRB.BINARY)

        model.setObjective(
            gp.quicksum(transport_cost[i][j] * x[i, j] for i in range(nn) for j in range(m)), GRB.MINIMIZE
        )

        for i in range(nn):
            model.addConstr(gp.quicksum(x[i, j] for j in range(m)) == 1)
        for j in range(m):
            model.addConstr(
                gp.quicksum(demands[customer_idx[i]] * x[i, j] for i in range(nn)) <= capacities[j] * y[j])
        for i in range(nn):
            for j in range(m):
                model.addConstr(x[i, j] <= y[j])

        model.optimize()

        assigned_customers = [[] for _ in range(m)]
        for i in range(nn):
            for j in range(m):
                if x[i, j].X > 0.5:
                    assigned_customers[j].append(customer_idx[i])
        return facility_coords, assigned_customers

    while True:
        s = time.time()
        facility_linehaul, assigned_linehaul = SSCFLP(time_left, linehaul_coord, linehaul_ids, kde_linehaul, mins_linehaul,
                                                  maxs_linehaul)
        time_left2 = s - time.time()
        facility_backhaul, assigned_backhaul = SSCFLP(time_left2, backhaul_coord, backhaul_ids, kde_backhaul, mins_backhaul,
                                                  maxs_backhaul)

        nonempty_linehaul_idx = [i for i, group in enumerate(assigned_linehaul) if group]
        nonempty_backhaul_idx = [j for j, group in enumerate(assigned_backhaul) if group]
        if len(nonempty_linehaul_idx) >= len(nonempty_backhaul_idx):
            break
        # else:
        #     print("라인홀 적음")

    def closest_pair_cost(i, j):
        nodes_l = assigned_linehaul[i]
        nodes_b = assigned_backhaul[j]
        return min(euclidean(coords_dict[u], coords_dict[v]) for u in nodes_l for v in nodes_b)

    cost_matrix = np.array([[closest_pair_cost(i, j) for j in nonempty_backhaul_idx] for i in nonempty_linehaul_idx])
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    match_pairs = [(nonempty_linehaul_idx[i], nonempty_backhaul_idx[j]) for i, j in zip(row_ind, col_ind)]

    best_routes_linehaul = [[] for _ in range(m)]
    for i in range(m):
        route = assigned_linehaul[i]
        _, result_route = two_opt_wb(depot_idx, route, dist_matrix, nodes_coord=False, show=False)
        best_routes_linehaul[i] = result_route

    best_routes_backhaul = [[] for _ in range(m)]
    for j in range(m):
        route = assigned_backhaul[j]
        _, result_route = two_opt_wb(depot_idx, route, dist_matrix, nodes_coord=False, show=False)
        best_routes_backhaul[j] = result_route

    def best_concatenation(l_route, b_route):
        cases = [
            l_route + b_route,
            list(reversed(l_route)) + b_route,
            l_route + list(reversed(b_route)),
            list(reversed(l_route)) + list(reversed(b_route))
        ]
        costs = []
        for case in cases:
            cost = sum(dist_matrix[case[i]][case[i + 1]] for i in range(len(case) - 1))
            costs.append(cost)
        return cases[np.argmin(costs)]

    best_result_route = [[] for _ in range(m)]
    for v_line, v_back in match_pairs:
        trimmed_l = best_routes_linehaul[v_line][1:-1]
        trimmed_b = best_routes_backhaul[v_back][1:-1]
        route = best_concatenation(trimmed_l, trimmed_b)
        best_result_route[v_line] = [0] + route + [0]

    for i in range(m):
        if not best_result_route[i]:
            trimmed = best_routes_linehaul[i][1:-1]
            best_result_route[i] = [0] + trimmed + [0]

    total_cost = sum(dist_matrix[r[i]][r[i + 1]] for r in best_result_route if r for i in range(len(r) - 1))
    return total_cost, best_result_route


def plot_routes_with_node_types(routes, coords_dict, node_type, total_cost, match_pairs=None, line_routes=None,
                                back_routes=None):
    if isinstance(coords_dict, list):
        coords_dict = {i: coord for i, coord in enumerate(coords_dict)}

    plt.xlim(-1000, 1000)
    plt.ylim(min(ys) * 1.05, max(ys) * 1.05)

    plt.figure(figsize=(12, 6))
    colors = matplotlib.colormaps.get_cmap("tab20")

    for idx, route in enumerate(routes):
        route_coords = [coords_dict[i] for i in route]
        xs, ys = zip(*route_coords)
        plt.plot(xs, ys, linestyle='-', color=colors(idx % 20), linewidth=2, alpha=0.9, label=f"Route {idx + 1}")

    if match_pairs and line_routes and back_routes:
        for l_idx, b_idx in match_pairs:
            l_end = coords_dict[line_routes[l_idx][-2]]
            b_start = coords_dict[back_routes[b_idx][1]]
            plt.plot([l_end[0], b_start[0]], [l_end[1], b_start[1]], color='black', linewidth=2, linestyle='--',
                     alpha=0.8)
            # Draw backhaul to depot with dotted line
            b_tail = coords_dict[back_routes[b_idx][-2]]
            depot = coords_dict[0]
            plt.plot([b_tail[0], depot[0]], [b_tail[1], depot[1]], linestyle=':', color='gray', linewidth=1.5)

    for i, (x, y) in coords_dict.items():
        node_t = node_type[i] if isinstance(node_type, (list, tuple)) else node_type.get(i, 0)
        if node_t == 0:
            plt.plot(x, y, marker='*', color='red', markersize=18, label='Depot' if i == 0 else "")
        elif node_t == 1:
            plt.plot(x, y, 'bo', markersize=10, label='Linehaul' if i == 1 else "")
        elif node_t == 2:
            plt.plot(x, y, 'r^', markersize=10, label='Backhaul' if i == 2 else "")

    plt.title(f"PWB (Total cost: {total_cost})")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(fontsize=8, loc="upper right", bbox_to_anchor=(1.25, 1))
    plt.tight_layout()
    plt.show()


# ---- facility별 TSP (depot 포함) 반복 최적화 및 최적 결과 저장 ----
def Iteration_VRPB(problem_info, time_limit, start_time, capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord,
                   backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul, kde_backhaul,
                   mins_backhaul, maxs_backhaul, n, m, vehicle_capacity, demands, node_types, coords_dict, dist_matrix,
                   show=False):
    best_cost = float('inf')
    #    best_facility_coords = None
    #    best_assigned_customers = None
    best_routes = None
    # i = 0
    while True:
        time_left = time_limit - time.time()
        total_cost, routes = run_sscflp_vrpb(time_left, capacities, depot_idx, depot_coord, linehaul_ids, linehaul_coord,
                                             backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul, maxs_linehaul,
                                             kde_backhaul, mins_backhaul, maxs_backhaul, n, m, vehicle_capacity,
                                             demands, node_types, coords_dict, dist_matrix)

        # if check_feasible_wb(problem_info, routes, 0, 1) == 0:
        #     continue

        if total_cost < best_cost:
            print(f"✅ Improved: {best_cost:.2f} → {total_cost} ({time.time() - start_time:.2f}s)")
            best_cost = total_cost
            #            best_facility_coords = facility_coords
            #            best_assigned_customers = assigned_customers
            best_routes = routes
        # if i % 100 == 0:
        #     print(f"Iteration {i} (elapsed: {time.time() - start_time:.1f}s)")
        # i += 1

        if time.time() - start_time > time_limit:
               # print(f"\n⏱️ Time limit of {time_limit} seconds reached after {i} iterations.")
               break

    return best_cost, best_routes


if __name__ == "__main__":
    # 예: instances 폴더 아래 problem_20_0.7.json 읽기
    instance_path = "../instances/problem_30_0.7.json"  # 경로는 본인 위치에 맞게 조정!
    with open(instance_path, "r") as f:
        data = json.load(f)
    n = data["N"]
    m = data["K"]
    vehicle_capacity = data["capa"]
    demands = data["node_demands"]
    node_types = data["node_types"]
    coords_dict = data["node_coords"]
    dist_matrix = data["dist_mat"]

    capacities = [vehicle_capacity] * m
    depot_idx = 0
    depot_coord = coords_dict[0]
    linehaul_ids = [i for i, t in enumerate(node_types) if t == 1]
    linehaul_coord = [coords_dict[i] for i in linehaul_ids]
    backhaul_ids = [i for i, t in enumerate(node_types) if t == 2]
    backhaul_coord = [coords_dict[i] for i in backhaul_ids]

    start_time = time.time()

    # ---- Customer 분포 주청 ----
    kde_linehaul = gaussian_kde(np.array(linehaul_coord).T)
    mins_linehaul = np.min(linehaul_coord, axis=0)
    maxs_linehaul = np.max(linehaul_coord, axis=0)
    kde_backhaul = gaussian_kde(np.array(backhaul_coord).T)
    mins_backhaul = np.min(backhaul_coord, axis=0)
    maxs_backhaul = np.max(backhaul_coord, axis=0)

if __name__ == "__main__":
    start_time = time.time()
    best_cost, best_routes = Iteration_VRPB(start_time, capacities, depot_idx, depot_coord, linehaul_ids,
                                            linehaul_coord, backhaul_ids, backhaul_coord, kde_linehaul, mins_linehaul,
                                            maxs_linehaul, kde_backhaul, mins_backhaul, maxs_backhaul, n, m,
                                            vehicle_capacity, demands, node_types, coords_dict, dist_matrix, show=False)
    print(f"\n✅ 전체 실행 시간: {time.time() - start_time:.2f}초")
    print(best_cost)
    print(best_routes)
    plot_routes_with_node_types(best_routes, coords_dict, node_types)


def check_feasible_wb(problem_info, sol, elapsed, timelimit):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    if elapsed > timelimit + 1:
        print("Time Out")
        return 0

    if len(sol) > K:
        print(f"vehicle 수는 {K}대까지 사용 가능합니다. 현재 : {len(sol)}")
        return 0

    total_cost = 0
    visit = [0] * len(node_type)
    visit[0] = 1

    for idx, route in enumerate(sol):
        if route[0] != 0:
            print(f"depot에서 출발해야 합니다. {idx}번째 차량 경로 {route}")
            return 0

        if route[-1] != 0:
            print(f"depot으로 도착해야 합니다. {idx}번째 차량 경로 {route}")
            return 0
        if node_type[route[1]] == 2:
            print(f"차량은 backhaul만 갈 수 없습니다. {idx}번째 차량 경로 {route}")
            return 0
        cost = 0
        load = 0

        pre = 0
        flag = False
        route_type = [0] * len(route)

        for i in range(1, len(route) - 1):
            nxt = route[i]
            if visit[nxt]:
                print(f"{nxt} 노드 2번 이상 방문")
                return 0
            visit[nxt] = 1
            cost += dist_mat[pre][nxt]
            load += node_demand[nxt]
            route_type[i] = node_type[nxt]
            if nxt == 0:
                print(f"{idx}번째 차량 depot을 3번 이상 방문 {route}")
                return 0

            if node_type[pre] == 1 and node_type[nxt] == 2:
                if flag:
                    print(f"{idx}번째 차량 line -> backhaul -> line 경로 존재")
                    print(node_type)
                    return 0
                flag = True
                load = 0

            if load > capa:
                if flag:
                    print(f"{idx}번째 차량의 line 용량 초과 {load}, {capa}")
                else:
                    print(f"{idx}번째 차량의 back 용량 초과 {load}, {capa}")
                return 0
            pre = nxt
        cost += dist_mat[pre][0]
        total_cost += cost

    if sum(visit) < len(node_type):
        yet_visit = [i for i in range(len(node_type)) if not visit[i]]
        print(f"다음 노드들을 방문하지 않았습니다. {yet_visit}")
        return 0

    return total_cost


def plot_vrpb_wb(problem_info, best_result_route=[], title=''):
    nodes_coord = problem_info['node_coords']
    nodes_type = problem_info['node_types']
    for i in range(len(nodes_type)):
        if nodes_type[i] == 0:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=150, marker='s', color='black')
        elif nodes_type[i] == 1:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=35, color='blue')
        else:
            plt.scatter(nodes_coord[i][0], nodes_coord[i][1], s=35, marker='^', color='red')

    for route in best_result_route:
        points_x = [nodes_coord[x][0] for x in route]
        points_y = [nodes_coord[x][1] for x in route]
        # plt.scatter(points_x, points_y)
        plt.plot([points_x[i] for i in range(len(route))], [points_y[i] for i in range(len(route))], linestyle='-',
                 label='Line', )
    plt.title(title)
    plt.xticks([])  # x축 눈금 없애기
    plt.yticks([])  # y축 눈금 없애기
    plt.show()
