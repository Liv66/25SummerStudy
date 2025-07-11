import random
import matplotlib.pyplot as plt
import gurobipy as gp
import time
from gurobipy import GRB
import numpy as np
from scipy.stats import gaussian_kde

from util_pure import two_opt, three_opt
from collections import Counter


# ---- 데이터 생성 ----
n = 100
m = 12
area_x, area_y = 24000, 32000
demand_mean, demand_std = 500, 200
facility_capacity = 5000

depot_coord = (12000, 16000)
customer_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(n)]
#minx, miny = np.array(customer_coords).min(axis=0)
#maxx, maxy = np.array(customer_coords).max(axis=0)
#facility_coords = [(random.uniform(minx, maxx), random.uniform(miny, maxy)) for _ in range(m)]
demands = [max(1, int(random.gauss(demand_mean, demand_std))) for _ in range(n)]
capacities = [facility_capacity for _ in range(m)]

# ---- 거리행렬 생성 (Depot + Customers) ----
def euclidean(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5
all_coords = [depot_coord] + customer_coords  # 0번이 depot, 1~n이 customer
dist_matrix = [
    [euclidean(all_coords[i], all_coords[j]) for j in range(n+1)]
    for i in range(n+1)
]

start_time = time.time()

# ---- Customer 분포 주청 ----
kde = gaussian_kde(np.array(customer_coords).T)
mins = np.min(customer_coords, axis=0)
maxs = np.max(customer_coords, axis=0)

# ---- ITERATION ----
def SSCFLP():
    # 클리핑 (범위 밖이면 min/max로 잘라냄)
    raw_samples = kde.resample(m).T  # shape: (m, 2)
    clipped_samples = np.clip(raw_samples, mins, maxs)
    facility_coords = [tuple(coord) for coord in clipped_samples]

    # ---- SSCFLP 구하기 ----
    transport_cost = [
        [euclidean(customer_coords[i], facility_coords[j]) for j in range(m)]
        for i in range(n)
    ]
    model = gp.Model("SSCFLP")
    model.setParam("OutputFlag", 0)
    model.setParam("LogToConsole", 0)
    model.setParam("TimeLimit", 50)
    x = model.addVars(n, m, vtype=GRB.BINARY, name="x")
    y = model.addVars(m, vtype=GRB.BINARY, name="y")
    model.setObjective(
        gp.quicksum(transport_cost[i][j]*x[i, j] for i in range(n) for j in range(m)),
        GRB.MINIMIZE
    )
    for i in range(n):
        model.addConstr(gp.quicksum(x[i, j] for j in range(m)) == 1)
    for j in range(m):
        model.addConstr(gp.quicksum(demands[i]*x[i, j] for i in range(n)) <= capacities[j]*y[j])
    for i in range(n):
        for j in range(m):
            model.addConstr(x[i, j] <= y[j])

    model.optimize()

    # ---- 할당결과로 각 facility별 고객 파티션 ----
    assigned_customers = [[] for _ in range(m)]
    for i in range(n):
        for j in range(m):
            if x[i, j].X > 0.5:
                assigned_customers[j].append(i)

    return facility_coords, assigned_customers


def TSP_result(show=False):
    total_cost = 0
    routes = []

    for fac_idx, cust_list in enumerate(assigned_customers):
        if not cust_list:
            routes.append([])  # 빈 루트라도 유지
            continue

        # customer_coords[i]는 all_coords[i+1]에 위치함 → 고객 → 노드 인덱스 변환
        node_indices = [i + 1 for i in cust_list]  # depot은 0, 고객은 1~
        depot_idx = 0

        # 3-opt로 depot 포함 루트 생성
        cost, result_route = two_opt(depot_idx, node_indices, dist_matrix, all_coords, show=show)
        total_cost += cost

        # depot(0) 제거하고 고객 인덱스 추출 → all_coords 기준 → customer_coords 인덱스로 보정 (x - 1)
        routes.append(result_route)
    return total_cost, routes



# ---- facility별 TSP (depot 포함) 반복 최적화 및 최적 결과 저장 ----
best_cost = float('inf')
best_facility_coords = None
best_assigned_customers = None
best_routes = None
time_limit = 59
i = 0
while True:
    facility_coords, assigned_customers = SSCFLP()
    total_cost, routes = TSP_result()

    if total_cost < best_cost:
        print(f"✅ Improved: {best_cost:.2f} → {total_cost:.2f}")
        best_cost = total_cost
        best_facility_coords = facility_coords
        best_assigned_customers = assigned_customers
        best_routes = routes

    if i % 100 == 0:
        print(f"Iteration {i} (elapsed: {time.time() - start_time:.1f}s)")

    i += 1
    if time.time() - start_time > time_limit:
        print(f"\n⏱️ Time limit of {time_limit} seconds reached after {i} iterations.")
        break

def save_solution(best_routes, best_cost, filename="output.sol"):
    with open(filename, "w") as f:
        for idx, route in enumerate(best_routes):
            if not route:
                continue  # 빈 route는 생략
            # customer 인덱스는 이미 0-based라고 가정
            route_str = ' '.join(str(cust) for cust in route)
            f.write(f"Route #{idx+1}: {route_str}\n")
        f.write(f"Cost {int(round(best_cost))}\n")

save_solution(best_routes, best_cost)

# ======== 좌표 딕셔너리 구성 (depot=0, customer=1~n) =========
coords_dict = {0: depot_coord}
for i, c in enumerate(customer_coords, start=1):
    coords_dict[i] = c

# ======== 시각화 함수 정의 =========
def visualize_solution(coords_dict, routes, depot_idx):
    plt.figure(figsize=(10, 8))
    colors = plt.cm.tab10(np.linspace(0, 1, len(routes)))
    for idx, route in enumerate(routes):
        # 각 번호를 1-based 그대로 딕셔너리에서 참조
        route_coords = [coords_dict[n] for n in route]
        full_route = [coords_dict[depot_idx]] + route_coords + [coords_dict[depot_idx]]
        xs, ys = zip(*full_route)
        plt.plot(xs, ys, marker='o', color=colors[idx], label=f"Route {idx+1}", linewidth=2)
    x_depot, y_depot = coords_dict[depot_idx]
    plt.scatter(x_depot, y_depot, color='red', s=200, label='Depot', marker='*', zorder=5)
    plt.title(f"Routes Visualization", fontsize=15)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

visualize_solution(coords_dict, best_routes, depot_idx=0)

print(f"\n✅ 전체 실행 시간: {time.time() - start_time:.2f}초")

# ======== 5. 해 검증 =========

def read_routes_from_file(filename):
    routes = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # "Route #1: 1214 319 196 376" 형태에서 번호 뒤 부분만 추출
            if line.startswith("Route"):
                # ':' 이후 텍스트만 추출
                parts = line.split(":", 1)
                if len(parts) < 2:
                    continue
                route_str = parts[1].strip()
                if route_str == "":
                    routes.append([])
                else:
                    nodes = list(map(int, route_str.split()))
                    routes.append(nodes)
    return routes

def check_duplicates(routes):
    all_nodes = []
    for r in routes:
        all_nodes.extend(r)

    node_counts = Counter(all_nodes)
    duplicates = [node for node, count in node_counts.items() if count > 1]

    if duplicates:
        print(f"중복된 노드가 있습니다: {duplicates}")
    else:
        print("중복된 노드가 없습니다.")

if __name__ == "__main__":
    filename = "output.sol"
    routes = read_routes_from_file(filename)
    check_duplicates(routes)



