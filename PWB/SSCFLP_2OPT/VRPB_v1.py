import vrplib
import random
import matplotlib.pyplot as plt
import gurobipy as gp
import time
from gurobipy import GRB
import numpy as np
from scipy.stats import gaussian_kde
from scipy.optimize import linear_sum_assignment

import json
from operator import itemgetter

import random

from util_pure import two_opt
from collections import Counter



# 예: instances 폴더 아래 problem_20_0.7.json 읽기
instance_path = "../../instances/problem_100_0.7.json"  # 경로는 본인 위치에 맞게 조정!

with open(instance_path, "r") as f:
    data = json.load(f)

n, m, vehicle_capacity, demands, node_types, coords_dict, dist_matrix = itemgetter("N", "K", "capa", "node_demands", "node_types", "node_coords", "dist_mat")(data)
demands
capacities = [vehicle_capacity] * m
depot_idx = 0
depot_coord = coords_dict[0]
linehaul_ids = [i for i, t in enumerate(node_types) if t == 1]
linehaul_coord = [coords_dict[i] for i in linehaul_ids]
backhaul_ids = [i for i, t in enumerate(node_types) if t == 2]
backhaul_coord = [coords_dict[i] for i in backhaul_ids]

start_time = time.time()

# ---- Customer 분포 주청 ----
kde_linehaul = gaussian_kde(np.array(linehaul_coord ).T)
mins_linehaul = np.min(linehaul_coord , axis=0)
maxs_linehaul = np.max(linehaul_coord , axis=0)
kde_backhaul = gaussian_kde(np.array(backhaul_coord ).T)
mins_backhaul = np.min(backhaul_coord , axis=0)
maxs_backhaul = np.max(backhaul_coord , axis=0)

def euclidean(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) ** 0.5

# ---- ITERATION ----
def SSCFLP(customer_coords, customer_idx, kde, mins, maxs):
    nn = len(customer_coords)
    raw_samples = kde.resample(m).T
    clipped_samples = np.clip(raw_samples, mins, maxs)
    facility_coords = [tuple(coord) for coord in clipped_samples]

    transport_cost = [
        [euclidean(customer_coords[i], facility_coords[j]) for j in range(m)]
        for i in range(nn)
    ]
    time_limit = max(0.1, 59 + start_time - time.time())
    model = gp.Model("SSCFLP")
    model.setParam("OutputFlag", 0)
    model.setParam("LogToConsole", 0)
    model.setParam("TimeLimit", time_limit)
    x = model.addVars(nn, m, vtype=GRB.BINARY, name="x")
    y = model.addVars(m, vtype=GRB.BINARY, name="y")
    model.setObjective(
        gp.quicksum(transport_cost[i][j]*x[i, j] for i in range(nn) for j in range(m)),
        GRB.MINIMIZE
    )
    for i in range(nn):
        model.addConstr(gp.quicksum(x[i, j] for j in range(m)) == 1)
    for j in range(m):
        model.addConstr(
            gp.quicksum(
                # 이 부분을 아래처럼!
                demands[customer_idx[i]]*x[i, j] for i in range(nn)
            ) <= capacities[j]*y[j]
        )
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


facility_linehaul, assigned_linehaul = SSCFLP(linehaul_coord, linehaul_ids, kde_linehaul, mins_linehaul, maxs_linehaul)
facility_backhaul, assigned_backhaul = SSCFLP(backhaul_coord, backhaul_ids, kde_backhaul, mins_backhaul, maxs_backhaul)

#assigned_linehaul = [group for group in assigned_linehaul if group]
#assigned_backhaul = [group for group in assigned_backhaul if group]
nonempty_linehaul_idx = [i for i, group in enumerate(assigned_linehaul) if group]
nonempty_backhaul_idx = [j for j, group in enumerate(assigned_backhaul) if group]

rough_transport_cost = [
    [euclidean(facility_linehaul[i], facility_backhaul[j]) for j in nonempty_backhaul_idx]
    for i in range(m)
]
rough_transport_cost = np.array(rough_transport_cost)  # 꼭 numpy 변환!
row_ind, col_ind = linear_sum_assignment(rough_transport_cost)

matched_linehaul_idx = [nonempty_linehaul_idx[r] for r in row_ind]
matched_backhaul_idx = [nonempty_backhaul_idx[c] for c in col_ind]


rough_hmatch = np.vstack((matched_linehaul_idx , matched_backhaul_idx))
missing = [i for i in range(m) if i not in rough_hmatch[0]]

best_routes_linehaul = [[] for _ in range(m)]
total_cost_linehaul = 0
for i in range(m):
    route = assigned_linehaul[i]
    cost, result_route = two_opt(depot_idx, route, dist_matrix, nodes_coord=False, show=False)
    total_cost_linehaul += cost
    best_routes_linehaul[i] = result_route


best_routes_backhaul = [[] for _ in range(m)]
total_cost_backhaul = 0
for j in nonempty_backhaul_idx:
    route = assigned_backhaul[j]
    cost, result_route = two_opt(depot_idx, route, dist_matrix, nodes_coord=False, show=False)
    total_cost_backhaul += cost
    best_routes_backhaul[j] = result_route


first_nodes_linehaul, last_nodes_linehaul = [route[1] for route in best_routes_linehaul], [route[-2] for route in best_routes_linehaul]
first_nodes_backhaul, last_nodes_backhaul = [route[1] for route in  [route for route in best_routes_backhaul if route]], [route[-2] for route in  [route for route in best_routes_backhaul if route]]
linehaul_cand = np.stack((first_nodes_linehaul, last_nodes_linehaul), axis=1)
backhaul_cand = np.stack((first_nodes_backhaul, last_nodes_backhaul), axis=1)

rough_match = np.stack((matched_linehaul_idx , matched_backhaul_idx), axis=1)

def connectlineback(linehaul_cand, backhaul_cand):
    connect_cost = np.full((len(linehaul_cand), len(backhaul_cand)), np.inf)
    min_ids = np.full((len(linehaul_cand), len(backhaul_cand)), np.inf)

    for i in range(len(linehaul_cand)):
        for j in range(len(backhaul_cand)):
            d00 = euclidean(coords_dict[int(linehaul_cand[i][0])], coords_dict[int(backhaul_cand[j][0])])
            d10 = euclidean(coords_dict[int(linehaul_cand[i][1])], coords_dict[int(backhaul_cand[j][0])])
            d01 = euclidean(coords_dict[int(linehaul_cand[i][0])], coords_dict[int(backhaul_cand[j][1])])
            d11 = euclidean(coords_dict[int(linehaul_cand[i][1])], coords_dict[int(backhaul_cand[j][1])])
            distances = [d00, d10, d01, d11]
            min_idx = np.argmin(distances)
            connect_cost[i, j] = distances[min_idx]
            min_ids[i, j] = min_idx
    row, col = linear_sum_assignment(connect_cost)

    matched_linehaul_indices = []
    matched_backhaul_indices = []
    for i, j in zip(row, col):
        min_idx = int(min_ids[i, j])
        # 각 매칭에 대해 어떤 꼭짓점이 선택됐는지
        if min_idx == 0:
            matched_linehaul_indices.append(linehaul_cand[i][0])
            matched_backhaul_indices.append(backhaul_cand[j][0])
        elif min_idx == 1:
            matched_linehaul_indices.append(linehaul_cand[i][1])
            matched_backhaul_indices.append(backhaul_cand[j][0])
        elif min_idx == 2:
            matched_linehaul_indices.append(linehaul_cand[i][0])
            matched_backhaul_indices.append(backhaul_cand[j][1])
        elif min_idx == 3:
            matched_linehaul_indices.append(linehaul_cand[i][1])
            matched_backhaul_indices.append(backhaul_cand[j][1])
    vehicle_hmatchs = np.vstack((matched_linehaul_indices, matched_backhaul_indices))
    vehicle_matchs = np.stack((matched_linehaul_indices, matched_backhaul_indices), axis=1)
    return connect_cost, vehicle_matchs, vehicle_hmatchs, min_ids


connect_cost, vehicle_matches, vehicle_hmatchs, min_ids = connectlineback(linehaul_cand,backhaul_cand)

trimmed_linehaul = [r[1:-1] for r in best_routes_linehaul]
trimmed_backhaul = [r[1:-1] for r in best_routes_backhaul]



best_result_route = [[] for _ in range(m)]
for u in range(m):
    # vehicle_hmatchs[0]이 np.array라면 set()으로 변환
    if not any(node in vehicle_hmatchs[0] for node in trimmed_linehaul[u]):
        best_result_route[u] = [depot_idx] + trimmed_linehaul[u] + [depot_idx]




linehaul_merge_idxs = [i for i, group in enumerate(trimmed_linehaul) if any(node in group for node in vehicle_hmatchs[0])]
backhaul_merge_idxs = [i for i, group in enumerate(trimmed_backhaul) if any(node in group for node in vehicle_hmatchs[1])]
haul_matches = np.stack((linehaul_merge_idxs, backhaul_merge_idxs), axis=1)

for i, v in enumerate(linehaul_merge_idxs):
    min_id = int(min_ids[v, i])   # i가 매칭 pair 번호
    back_idx = backhaul_merge_idxs[i]  # 실제 trimmed_backhaul의 인덱스
    # 이하 병합 코드:
    if min_id == 0:
        best_result_route[v] = [depot_idx] + trimmed_linehaul[v] + trimmed_backhaul[back_idx] + [depot_idx]
    elif min_id == 1:
        best_result_route[v] = [depot_idx] + list(reversed(trimmed_linehaul[v])) + trimmed_backhaul[back_idx] + [depot_idx]
    elif min_id == 2:
        best_result_route[v] = [depot_idx] + trimmed_linehaul[v] + list(reversed(trimmed_backhaul[back_idx])) + [depot_idx]
    elif min_id == 3:
        best_result_route[v] = [depot_idx] + list(reversed(trimmed_linehaul[v])) + list(reversed(trimmed_backhaul[back_idx])) + [depot_idx]




def route_length(route, dist_matrix):
    # route는 [0, ..., 0] 형태의 노드 인덱스 리스트
    length = 0
    for i in range(len(route) - 1):
        a, b = route[i], route[i+1]
        length += dist_matrix[a][b]
    return length

# ---- 최적화 비용(총 거리) 계산 ----
total_cost = 0
for route in best_result_route:
    if not route:
        continue
    cost = route_length(route, dist_matrix)
    total_cost += cost
print("총 최적화 비용(총 거리):", total_cost)


print(f"\n✅ 전체 실행 시간: {time.time() - start_time:.2f}초")




def save_solution(best_routes, filename="output.sol"):
    with open(filename, "w") as f:
        for idx, route in enumerate(best_routes):
            if not route:
                continue  # 빈 route는 생략
            # customer 인덱스는 이미 0-based라고 가정
            route = route
            route_str = ' '.join(str(cust) for cust in route)
            f.write(f"Route #{idx+1}: {route_str}\n")

if __name__ == "__main__":
    save_solution(best_result_route, filename="output.sol")

