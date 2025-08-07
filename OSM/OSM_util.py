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
                original_dist = dist_matrix[result_route[i-1]][result_route[i]] + dist_matrix[result_route[j]][result_route[j+1]]
                new_dist = dist_matrix[result_route[i-1]][result_route[j]] + dist_matrix[result_route[i]][result_route[j+1]]
                if new_dist < original_dist:
                    result_route = result_route[:i] + result_route[i:j+1][::-1] + result_route[j+1:]
                    improved = True
    
    final_cost = sum(dist_matrix[result_route[i]][result_route[i + 1]] for i in range(len(result_route) - 1))
    return final_cost, result_route

"""
def two_opt(nodes, dist_matrix, demand_dict):   # 새로운거
    def is_valid_vrpb_route(route, demands):
        in_backhaul_phase = False
        for node_idx in route:
            if node_idx == 0: continue
            if demands.get(node_idx, 0) < 0:
                in_backhaul_phase = True
            if demands.get(node_idx, 0) > 0 and in_backhaul_phase:
                return False # Backhaul 후 Linehaul 방문 시 규칙 위반
        return True

    result_route = nodes[:]
    improved = True
    
    while improved:
        improved = False
        for i in range(1, len(result_route) - 2):
            for j in range(i + 1, len(result_route) - 1):
                
                # 1. 비용(거리) 변화를 먼저 계산
                original_dist = dist_matrix[result_route[i-1]][result_route[i]] + dist_matrix[result_route[j]][result_route[j+1]]
                new_dist = dist_matrix[result_route[i-1]][result_route[j]] + dist_matrix[result_route[i]][result_route[j+1]]

                # 2. 거리가 줄어드는 경우에만 경로를 만들고 검증
                if new_dist < original_dist:
                    new_route = result_route[:i] + result_route[i:j+1][::-1] + result_route[j+1:]
                    
                    # 3. 변경된 경로가 VRPB 규칙을 지키는지 최종 확인
                    if is_valid_vrpb_route(new_route, demand_dict):
                        result_route = new_route
                        improved = True
                        
    final_cost = sum(dist_matrix[result_route[i]][result_route[i + 1]] for i in range(len(result_route) - 1))
    return final_cost, result_route
"""
"""
def two_opt(nodes, dist_matrix, demand_dict):
    
    def is_valid_vrpb_route(route, demands):
        in_backhaul_phase = False
        for node_idx in route:
            if node_idx == 0: continue
            is_linehaul = demands.get(node_idx, 0) > 0
            is_backhaul = demands.get(node_idx, 0) < 0
            if is_backhaul: in_backhaul_phase = True
            if is_linehaul and in_backhaul_phase: return False
        return True

    result_route = nodes[:]
    improved = True
    
    while improved:
        improved = False
        for i in range(1, len(result_route) - 2):
            for j in range(i + 1, len(result_route) - 1):
                new_route = result_route[:i] + result_route[i:j+1][::-1] + result_route[j+1:]
                
                if is_valid_vrpb_route(new_route, demand_dict):
                    original_dist = dist_matrix[result_route[i-1]][result_route[i]] + dist_matrix[result_route[j]][result_route[j+1]]
                    new_dist = dist_matrix[new_route[i-1]][new_route[i]] + dist_matrix[new_route[j]][new_route[j+1]]
                    
                    if new_dist < original_dist:
                        result_route = new_route
                        improved = True
                        
    final_cost = sum(dist_matrix[result_route[i]][result_route[i + 1]] for i in range(len(result_route) - 1))
    return final_cost, result_route
"""
    
def get_distance(nodes_coord):
    """노드들의 좌표가 들어오면 거리 행렬(리스트의 리스트)을 반환합니다."""
    N = len(nodes_coord)
    return [[int(sqrt((nodes_coord[i][0] - nodes_coord[j][0]) ** 2 + (nodes_coord[i][1] - nodes_coord[j][1]) ** 2)) for j in range(N)] for i in range(N)]

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
                plt.scatter(coord[0], coord[1], c='green', s=60, label='Linehaul' if i==1 else "", zorder=2)
            elif demands[i] < 0:
                plt.scatter(coord[0], coord[1], c='blue', s=60, label='Backhaul' if demands.count(min(demands))==1 and demands[i]==min(demands) else "", zorder=2)
    
    # 차량별 경로 그리기
    color_list = ['red', 'orange', 'purple', 'brown', 'cyan', 'magenta', 'olive', 'pink', 'gray', 'gold']
    for i, route in enumerate(best_result_route):
        color = color_list[i % len(color_list)]
        full_route = [0] + route + [0]
        route_coords_x = [nodes_coord[node_idx][0] for node_idx in full_route]
        route_coords_y = [nodes_coord[node_idx][1] for node_idx in full_route]
        plt.plot(route_coords_x, route_coords_y, color=color, linewidth=1.5, marker='o', markersize=4, label=f'Vehicle {i+1}')

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    # plt.show()

def multiple_knapsack_vrpb(demands_dict, capa, K, nodes_coord):
    """
    CP-SAT 솔버를 사용하여 VRPB의 고객들을 K개의 그룹으로 나눕니다.
    (용량 제약 조건만 만족하는 유효한 해를 빠르게 찾는 데 집중)
    """
    customers = [k for k in demands_dict if k != 0]
    cust_to_idx = {cust_id: i for i, cust_id in enumerate(customers)}
    idx_to_cust = {i: cust_id for i, cust_id in enumerate(customers)}
    num_customers = len(customers)

    model = cp_model.CpModel()
    x = [[model.NewBoolVar(f'x_{i}_{j}') for j in range(K)] for i in range(num_customers)]

    for i in range(num_customers):
        model.AddExactlyOne(x[i][j] for j in range(K))

    for j in range(K):
        linehaul_load = sum(demands_dict[idx_to_cust[i]] * x[i][j] 
                            for i in range(num_customers) if demands_dict[idx_to_cust[i]] > 0)
        model.Add(linehaul_load <= capa)
        backhaul_load = sum(abs(demands_dict[idx_to_cust[i]]) * x[i][j] 
                            for i in range(num_customers) if demands_dict[idx_to_cust[i]] < 0)
        model.Add(backhaul_load <= capa)

    # --- 목표(Objective) 설정 부분 삭제 ---
    # model.Minimize(...) 부분을 제거하여, 제약 조건만 만족하는
    # '가능한 해(feasible solution)'를 빠르게 찾도록 함

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 10.0
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        clusters = [[] for _ in range(K)]
        for i in range(num_customers):
            for j in range(K):
                if solver.Value(x[i][j]) == 1:
                    clusters[j].append(idx_to_cust[i])
                    break
        return clusters
    else:
        print("Warning: Could not find a valid clustering solution.")
        return None
    
