import random
import matplotlib.pyplot as plt
import gurobipy as gp
import time
from gurobipy import GRB
import numpy as np
from util_pure import two_opt, three_opt
from collections import Counter


# ---- 데이터 생성 ----
n = 50
m = 6
area_x, area_y = 24000, 32000
demand_mean, demand_std = 500, 200
facility_capacity = 5000

depot_coord = (12000, 16000)
customer_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(n)]
minx, miny = np.array(customer_coords).min(axis=0)
maxx, maxy = np.array(customer_coords).max(axis=0)
facility_coords = [(random.uniform(minx, maxx), random.uniform(miny, maxy)) for _ in range(m)]
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

# ---- SSCFLP 구하기 ----
transport_cost = [
    [euclidean(customer_coords[i], facility_coords[j]) for j in range(m)]
    for i in range(n)
]
model = gp.Model("SSCFLP")
model.Params.TimeLimit = 50
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
model.Params.OutputFlag = 0
model.optimize()

# ---- 할당결과로 각 facility별 고객 파티션 ----
assigned_customers = [[] for _ in range(m)]
for i in range(n):
    for j in range(m):
        if x[i, j].X > 0.5:
            assigned_customers[j].append(i)

# ---- facility별 TSP (depot 포함) 최적화 및 결과 저장 ----
with open("output.sol", "w") as f:
    for fac_idx, cust_list in enumerate(assigned_customers):
        if not cust_list:
            continue
        # depot: 0, customers: +1 shift
        node_indices = [i+1 for i in cust_list]  # customer_coords[i]는 all_coords[i+1]에 해당
        depot_idx = 0  # all_coords에서 depot은 항상 0번
        nodes_coord = all_coords  # 0: depot, 1~n: customers
        cost, result_route = three_opt(depot_idx, node_indices, dist_matrix, nodes_coord, show=False)
        # result_route는 depot부터, 실제 고객 인덱스만 추출 (depot=0)
        route_customers = [x-1 for x in result_route[1:-1]]  # 0 제외, -1로 원래 customer
        f.write(f"Route #{fac_idx+1}: {' '.join(str(x) for x in route_customers)}\n")
print("Done. Output saved to output.sol.")
print(dist_matrix)
# ======== 1. output.sol에서 routes 읽어오기 =========
routes = []
with open("output.sol") as f:
    for line in f:
        if line.startswith("Route"):
            parts = line.strip().split(":")
            route = [int(x) for x in parts[1].split()]
            routes.append(route)

# ======== 2. 좌표 딕셔너리 구성 (depot=0, customer=1~n) =========
coords_dict = {0: depot_coord}
for i, c in enumerate(customer_coords, start=1):
    coords_dict[i] = c

# ======== 3. 시각화 함수 정의 =========
import matplotlib.pyplot as plt
import numpy as np

def visualize_solution(coords_dict, routes, depot_idx=0, instance_name="SSCFL-TSP Solution"):
    plt.figure(figsize=(10, 8))
    colors = plt.cm.tab20(np.linspace(0, 1, max(len(routes), 20)))
    for idx, route in enumerate(routes):
        if not route:  # skip empty route
            continue
        # depot(0)에서 출발, depot으로 돌아옴
        full_route = [depot_idx] + [x+1 for x in route] + [depot_idx]
        route_coords = [coords_dict[n] for n in full_route]
        xs, ys = zip(*route_coords)
        plt.plot(xs, ys, marker='o', color=colors[idx % 10], label=f"Route {idx+1}", linewidth=2)
    x_depot, y_depot = coords_dict[depot_idx]
    plt.scatter(x_depot, y_depot, color='red', s=200, label='Depot', marker='*', zorder=5)
    plt.title(f"Routes Visualization for {instance_name}", fontsize=15)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

# ======== 4. 시각화 실행 =========
visualize_solution(coords_dict, routes, depot_idx=0, instance_name="SSCFL-2OPT Solution")

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



