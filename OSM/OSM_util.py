from math import sqrt
import matplotlib.pyplot as plt
from ortools.sat.python import cp_model


def two_opt(nodes, dist_matrix):
    """
    주어진 노드 리스트(경로)를 2-opt 알고리즘으로 최적화합니다.
    (VRPB 규칙은 고려하지 않는 표준 버전)
    """
    result_route = nodes[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(result_route) - 2):
            for j in range(i + 1, len(result_route) - 1):
                original_dist = dist_matrix[result_route[i - 1]][result_route[i]] + dist_matrix[result_route[j]][
                    result_route[j + 1]]
                new_dist = dist_matrix[result_route[i - 1]][result_route[j]] + dist_matrix[result_route[i]][
                    result_route[j + 1]]
                if new_dist < original_dist:
                    result_route = result_route[:i] + result_route[i:j + 1][::-1] + result_route[j + 1:]
                    improved = True

    final_cost = sum(dist_matrix[result_route[i]][result_route[i + 1]] for i in range(len(result_route) - 1))
    return final_cost, result_route


def get_distance(nodes_coord):
    N = len(nodes_coord)
    return [
        [int(sqrt((nodes_coord[i][0] - nodes_coord[j][0]) ** 2 + (nodes_coord[i][1] - nodes_coord[j][1]) ** 2)) for i in
         range(N)] for j in
        range(N)]


def plot_cvrp(nodes_coord, best_result_route, title='', demands=None):
    """CVRP 결과를 시각화합니다."""
    plt.figure(figsize=(12, 8))
    depot_coord = nodes_coord[0]

    # 노드 종류별 마커 표시
    plt.scatter(depot_coord[0], depot_coord[1], c='black', marker='s', s=150, label='Depot', zorder=3)
    if demands:
        for i, coord in enumerate(nodes_coord):
            if i == 0: continue
            if demands[i] > 0:
                plt.scatter(coord[0], coord[1], c='green', s=60, label='Linehaul' if i == 1 else "", zorder=2)
            elif demands[i] < 0:
                plt.scatter(coord[0], coord[1], c='blue', s=60,
                            label='Backhaul' if demands.count(min(demands)) == 1 and demands[i] == min(demands) else "",
                            zorder=2)

    # 차량별 경로 그리기
    color_list = ['red', 'orange', 'purple', 'brown', 'cyan', 'magenta', 'olive', 'pink', 'gray', 'gold']
    for i, route in enumerate(best_result_route):
        color = color_list[i % len(color_list)]
        full_route = [0] + route + [0]
        route_coords_x = [nodes_coord[node_idx][0] for node_idx in full_route]
        route_coords_y = [nodes_coord[node_idx][1] for node_idx in full_route]
        plt.plot(route_coords_x, route_coords_y, color=color, linewidth=1.5, marker='o', markersize=4,
                 label=f'Vehicle {i + 1}')

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()


def check_feasible(problem_info, sol, elapsed, timelimit):
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
                    print(f"{idx}번째 차량의 back 용량 초과 {load}, {capa}")
                else:
                    print(f"{idx}번째 차량의 line 용량 초과 {load}, {capa}")
                return 0
            pre = nxt
        cost += dist_mat[pre][0]
        total_cost += cost

    if sum(visit) < len(node_type):
        yet_visit = [i for i in range(len(node_type)) if not visit[i]]
        print(f"다음 노드들을 방문하지 않았습니다. {yet_visit}")
        return 0

    return total_cost