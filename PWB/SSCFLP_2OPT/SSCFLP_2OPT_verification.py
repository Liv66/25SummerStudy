import vrplib
import random
import gurobipy as gp
import time
from gurobipy import GRB
import numpy as np

from test import facility_coords
from util_pure import two_opt, three_opt
from collections import Counter
import re
import matplotlib.pyplot as plt

# 문제 불러오기

instance_name = "A-n32-k5"
vrp_path = f"./vrp_instances/{instance_name}.vrp"
sol_path = f"./vrp_instances/{instance_name}.sol"

problem = vrplib.read_instance(vrp_path)
coords_dict = problem["node_coord"]  # {1: (x1, y1), 2: (x2, y2), ...}
depot = problem["depot"]

n = 32
m = 5
# depot 제외한 demands (첫 번째 요소 제외)
demands = problem.get("demand", [1] * n)[1:]  # depot 제외

# Vehicle capacity 추가 (VRP 문제에서 가져오기)
vehicle_capacity = problem.get("capacity", 100)  # 기본값 100
capacities = [vehicle_capacity] * m  # 모든 facility가 같은 capacity를 가진다고 가정

# depot robust 변환 (보통 1)
if isinstance(depot, (list, tuple, np.ndarray)):
    depot_idx = int(np.asarray(depot).item())
else:
    depot_idx = int(depot)
depot_coord = coords_dict[depot_idx]
# 만약 depot_idx가 0 이거나 0-based 인덱스라면 1-based로 맞추기
if depot_idx == 0:
    depot_idx = 1  # 보통 depot은 1번 노드인 경우가 많음
# 시각화
# coords_dict 생성 (1-based 키)
coords_dict = {i + 1: tuple(problem["node_coord"][i]) for i in range(len(problem["node_coord"]))}

# depot 좌표 추출 (1-based 키 사용)
x_depot, y_depot = coords_dict[depot_idx]

# 시각화
# plt.figure(figsize=(8, 6))
customer_ids = [k for k in coords_dict.keys() if k != depot_idx]
coords_array = problem["node_coord"]  # shape (n, 2), 1-based 인덱스에 맞춰서
minx, miny = coords_array.min(axis=0)
maxx, maxy = coords_array.max(axis=0)
customer_coords = [coords_dict[k] for k in customer_ids]
facility_coords = [(random.uniform(minx, maxx), random.uniform(miny, maxy)) for _ in range(m)]
# xs, ys = zip(*customer_coords)
# plt.scatter(xs, ys, c='orange', label='Customers', s=40)
# plt.scatter(x_depot, y_depot, c='red', label='Depot', s=100, marker='*')
# plt.title(f"Node Coordinates - Depot {depot_idx}")
# plt.xlabel("X coordinate")
# plt.ylabel("Y coordinate")
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.show()

# 거리 행렬 생성: depot 포함 모든 노드 거리 계산
n = coords_array.shape[0]


def euclidean(p1, p2):
    p1 = np.array(p1)  # 이 줄이 빠져있음!
    p2 = np.array(p2)  # 이 줄이 빠져있음!
    return np.linalg.norm(p1 - p2)


dist_matrix = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        dist_matrix[i, j] = euclidean(coords_array[i], coords_array[j])

print(f"Distance matrix shape: {dist_matrix.shape}")

# dist_matrix는 0-based 인덱스 (0~n-1)
# depot_idx가 1-based면, 실제 인덱스는 depot_idx-1임

print(f"Depot index (0-based): {depot_idx - 1}")
print(f"Depot coordinates: {coords_array[depot_idx - 1]}")

start_time = time.time()

# ---- SSCFLP 구하기 ----
# customer 개수 사용
n_customers = len(customer_coords)  # 31

print(f"Number of customers: {n_customers}")
print(f"Number of facilities: {m}")
print(f"Demands length: {len(demands)}")

transport_cost = [
    [euclidean(customer_coords[i], facility_coords[j]) for j in range(m)]
    for i in range(n_customers)  # range(31) 사용
]

model = gp.Model("SSCFLP")
model.Params.TimeLimit = 50
# 모델 변수들도 customer 개수로 수정
x = model.addVars(n_customers, m, vtype=GRB.BINARY, name="x")
y = model.addVars(m, vtype=GRB.BINARY, name="y")

model.setObjective(
    gp.quicksum(transport_cost[i][j] * x[i, j] for i in range(n_customers) for j in range(m)),
    GRB.MINIMIZE
)

for i in range(n_customers):
    model.addConstr(gp.quicksum(x[i, j] for j in range(m)) == 1)

for j in range(m):
    model.addConstr(gp.quicksum(demands[i] * x[i, j] for i in range(n_customers)) <= capacities[j] * y[j])

for i in range(n_customers):
    for j in range(m):
        model.addConstr(x[i, j] <= y[j])
model.Params.OutputFlag = 0
model.optimize()

# 최적화 상태 확인
if model.Status == GRB.OPTIMAL:
    print("최적화 성공!")
elif model.Status == GRB.INFEASIBLE:
    print("문제가 불가능합니다.")
elif model.Status == GRB.TIME_LIMIT:
    print("시간 제한에 도달했습니다.")
else:
    print(f"최적화 상태: {model.Status}")

# ---- 할당결과로 각 facility별 고객 파티션 ----
assigned_customers = [[] for _ in range(m)]
if model.Status in [GRB.OPTIMAL, GRB.TIME_LIMIT]:
    for i in range(n_customers):  # 0부터 30까지 (총 31개 고객)
        for j in range(m):
            if x[i, j].X > 0.5:
                assigned_customers[j].append(i)

    print("할당 결과:")
    for j, customers in enumerate(assigned_customers):
        if customers:
            total_demand = sum(demands[i] for i in customers)
            print(f"Facility {j}: {len(customers)} customers, total demand: {total_demand}")
else:
    print("최적화에 실패했습니다. 프로그램을 종료합니다.")
    exit()

# all_coords 정의 추가 (depot + customers)
all_coords = [depot_coord] + customer_coords

# ---- facility별 TSP (depot 포함) 최적화 및 결과 저장 ----
with open("output.sol", "w") as f:
    for fac_idx, cust_list in enumerate(assigned_customers):
        if not cust_list:
            continue
        # depot: 0, customers: +1 shift
        node_indices = [i + 1 for i in cust_list]  # customer_coords[i]는 all_coords[i+1]에 해당
        depot_idx_tsp = 0  # all_coords에서 depot은 항상 0번
        nodes_coord = all_coords  # 0: depot, 1~n: customers
        cost, result_route = three_opt(depot_idx_tsp, node_indices, dist_matrix, nodes_coord, show=False)
        # result_route는 depot부터, 실제 고객 인덱스만 추출 (depot=0)
        route_customers = [x - 1 for x in result_route[1:-1]]  # 0 제외, -1로 원래 customer index
        f.write(f"Route #{fac_idx + 1}: {' '.join(str(x) for x in route_customers)}\n")

print("Done. Output saved to output.sol.")

# ======== 1. output.sol에서 routes 읽어오기 =========
routes = []
with open("output.sol") as f:
    for line in f:
        if line.startswith("Route"):
            parts = line.strip().split(":")
            route = [int(x) for x in parts[1].split()]
            routes.append(route)

# ======== 2. 좌표 딕셔너리 구성 (depot=0, customer=1~n) =========
coords_dict_viz = {0: depot_coord}
for i, c in enumerate(customer_coords, start=1):
    coords_dict_viz[i] = c

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
        full_route = [depot_idx] + [x + 1 for x in route] + [depot_idx]
        route_coords = [coords_dict[n] for n in full_route]
        xs, ys = zip(*route_coords)
        plt.plot(xs, ys, marker='o', color=colors[idx % 10], label=f"Route {idx + 1}", linewidth=2)
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
visualize_solution(coords_dict_viz, routes, depot_idx=0, instance_name="SSCFL-2OPT Solution")

print(f"\n✅ 전체 실행 시간: {time.time() - start_time:.2f}초")


# ======== 5. 해 검증 및 거리 계산 =========

def calculate_route_distance(route, coords_dict, depot_idx=0):
    """주어진 route의 총 거리를 계산"""
    if not route:
        return 0

    full_route = [depot_idx] + [x + 1 for x in route] + [depot_idx]
    total_dist = 0

    for i in range(len(full_route) - 1):
        p1 = coords_dict[full_route[i]]
        p2 = coords_dict[full_route[i + 1]]
        total_dist += euclidean(p1, p2)

    return total_dist


def read_routes_from_file(filename):
    routes = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # "Route #1: 1214 319 196 376 (Distance: 123.45)" 형태에서 번호 뒤 부분만 추출
            if line.startswith("Route"):
                # ':' 이후 텍스트만 추출
                parts = line.split(":", 1)
                if len(parts) < 2:
                    continue
                route_str = parts[1].strip()

                # Distance 정보가 있다면 제거
                if "(Distance:" in route_str:
                    route_str = route_str.split("(Distance:")[0].strip()

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
        print(f"❌ 중복된 노드가 있습니다: {duplicates}")
        return False
    else:
        print("✅ 중복된 노드가 없습니다.")
        return True


def verify_solution(routes, coords_dict, depot_idx=0):
    """해의 유효성을 검증하고 총 거리를 계산"""
    print(f"\n🔍 해 검증 결과:")

    # 중복 검사
    is_valid = check_duplicates(routes)

    # 거리 계산
    total_verified_distance = 0
    for i, route in enumerate(routes):
        if route:
            dist = calculate_route_distance(route, coords_dict, depot_idx)
            total_verified_distance += dist
            print(f"Route {i + 1}: {len(route)} customers, distance: {dist:.2f}")

    print(f"\n🎯 검증된 총 이동거리: {total_verified_distance:.2f}")
    return is_valid, total_verified_distance


if __name__ == "__main__":
    filename = "output.sol"
    routes = read_routes_from_file(filename)
    is_valid, verified_distance = verify_solution(routes, coords_dict_viz, depot_idx=0)

    print(f"\n📊 최종 결과 요약:")
    print(f"- 해가 유효한가: {'✅ 예' if is_valid else '❌ 아니오'}")
    print(f"- 총 route 수: {len([r for r in routes if r])}")
    print(f"- 총 고객 수: {sum(len(r) for r in routes)}")
    print(f"- 최종 총 이동거리: {verified_distance:.2f}")
