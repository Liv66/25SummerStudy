import numpy as np
import matplotlib.pyplot as plt
import time
import random
import os
from scipy.spatial.distance import cdist
from itertools import permutations

# -------------------------------
# 인스턴스 파라미터 설정
# -------------------------------
n = 30
m = 5
area_x, area_y = 24000, 32000
demand_mean, demand_std = 500, 200
facility_capacity = 5000

depot_coord = (12000, 16000)

# -------------------------------
# 고객 및 시설 좌표 생성
# -------------------------------
customer_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(n)]
facility_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(m)]

# -------------------------------
# 고객 수요 및 시설 데이터 생성
# -------------------------------
demands = [max(1, int(random.gauss(demand_mean, demand_std))) for _ in range(n)]
capacities = [facility_capacity for _ in range(m)]
fixed_costs = [random.randint(1000, 3000) for _ in range(m)]

# -------------------------------
# 거리 기반 비용 행렬
# -------------------------------
def compute_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

cost_matrix = np.zeros((m, n))
for i in range(m):
    for j in range(n):
        cost_matrix[i][j] = int(compute_distance(facility_coords[i], customer_coords[j]))

# -------------------------------
# Knapsack Solver
# -------------------------------
def solve_knapsack(i, u, b_i):
    cost = cost_matrix[i] - u
    ratio = cost / np.array(demands)
    idx = np.argsort(ratio)
    x = np.zeros(n, dtype=int)
    total = 0
    for j in idx:
        if cost[j] < 0 and total + demands[j] <= b_i:
            x[j] = 1
            total += demands[j]
    return x

# -------------------------------
# Subgradient
# -------------------------------
def compute_subgradient(x_sol):
    return 1 - np.sum(x_sol, axis=0)

def step_length(gap, d, kappa=1.0):
    norm_sq = np.sum(d ** 2)
    return 0 if norm_sq == 0 else kappa * gap / norm_sq

# -------------------------------
# Feasibility Heuristic
# -------------------------------
def make_feasible(x_sol, y_sol, b_local):
    assigned = np.sum(x_sol, axis=0)
    for j in range(n):
        if assigned[j] == 0:
            best, best_cost = None, float('inf')
            for i in range(m):
                if b_local[i] >= demands[j] and cost_matrix[i][j] < best_cost:
                    best, best_cost = i, cost_matrix[i][j]
            if best is not None:
                x_sol[best][j] = 1
                y_sol[best] = 1
                b_local[best] -= demands[j]
            else:
                return None
    return x_sol, y_sol

# -------------------------------
# Lagrangian Heuristic
# -------------------------------
def lagrangian_heuristic():
    u = np.zeros(n)
    best_upper = float('inf')
    best_lower = -float('inf')

    for it in range(100):
        x_sol = np.zeros((m, n), dtype=int)
        y_sol = np.zeros(m, dtype=int)
        b_local = capacities.copy()

        for i in range(m):
            x_i = solve_knapsack(i, u, b_local[i])
            x_sol[i] = x_i
            if np.any(x_i):
                y_sol[i] = 1
                b_local[i] -= np.sum(np.array(demands) * x_i)

        g_val = np.sum((cost_matrix - u) * x_sol) + np.sum(np.array(fixed_costs) * y_sol) + np.sum(u)
        best_lower = max(best_lower, g_val)

        feasible = make_feasible(x_sol.copy(), y_sol.copy(), capacities.copy())
        if feasible is None:
            continue
        x_feas, y_feas = feasible
        upper = np.sum(cost_matrix * x_feas) + np.sum(np.array(fixed_costs) * y_feas)
        best_upper = min(best_upper, upper)

        d = compute_subgradient(x_sol)
        gap = best_upper - g_val
        t = step_length(gap, d)
        u = u + t * d

        print(f"Iter {it:2d} | Lower: {g_val:.2f} | Upper: {best_upper:.2f} | Gap: {gap:.2f}")
        if gap < 1:
            break

    return x_feas, y_feas

# -------------------------------
# Extract Clusters
# -------------------------------
def extract_clusters(x_sol, y_sol):
    clusters = []
    for i in range(m):
        if y_sol[i]:
            assigned = [j for j in range(n) if x_sol[i][j]]
            if assigned:
                clusters.append((i, assigned))
    return clusters

# -------------------------------
# Nearest Neighbor TSP Solver (Heuristic)
# -------------------------------
def nearest_neighbor_tsp(points):
    n = len(points)
    visited = [False] * n
    order = [0]
    visited[0] = True
    for _ in range(n - 1):
        last = order[-1]
        next_city = min((i for i in range(n) if not visited[i]), key=lambda i: compute_distance(points[last], points[i]))
        order.append(next_city)
        visited[next_city] = True
    order.append(0)
    total_dist = sum(compute_distance(points[order[i]], points[order[i+1]]) for i in range(len(order) - 1))
    return order, total_dist

# -------------------------------
# 시각화 함수
# -------------------------------
def plot_tsp_route(points, order, color, title="TSP Tour"):
    xs = [points[i][0] for i in order]
    ys = [points[i][1] for i in order]
    plt.plot(xs, ys, 'o-', color=color, alpha=0.7)
    plt.scatter(xs[0], ys[0], color='green', s=200, label='Depot')
    plt.scatter(xs[1:-1], ys[1:-1], color=color, s=50, label='Customer')
    plt.title(title)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')

# -------------------------------
# 실행부
# -------------------------------
start_time = time.time()
result = lagrangian_heuristic()
if result is None:
    print("No feasible solution found.")
    exit()

x_final, y_final = result
clusters = extract_clusters(x_final, y_final)

plt.figure(figsize=(10, 10))
colors = plt.cm.tab20(np.linspace(0, 1, len(clusters)))

for idx, (facility, customers) in enumerate(clusters):
    cluster_coords = [depot_coord] + [customer_coords[j] for j in customers]
    route, length = nearest_neighbor_tsp(cluster_coords)
    plot_tsp_route(cluster_coords, route, colors[idx], title=f"Facility {facility} | Length: {length:.1f}")

plt.suptitle("All TSP Routes (Heuristic)")
plt.show()

print(f"\n✅ 전체 실행 시간: {time.time() - start_time:.2f}초")
