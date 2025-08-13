import numpy as np
import matplotlib.pyplot as plt
import time
import random
from util_pure import two_opt

# -------------------------------
# 인스턴스 파라미터
# -------------------------------
n = 150
m = 20
area_x, area_y = 24000, 32000
demand_mean, demand_std = 500, 200
facility_capacity = 5000
depot_coord = (12000, 16000)

# -------------------------------
# 고객 및 시설 좌표, 수요 생성
# -------------------------------
customer_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(n)]
facility_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(m)]
demands = [max(1, int(random.gauss(demand_mean, demand_std))) for _ in range(n)]
capacities = [facility_capacity for _ in range(m)]
fixed_costs = [random.randint(1000, 3000) for _ in range(m)]

def compute_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

cost_matrix = np.zeros((m, n))
for i in range(m):
    for j in range(n):
        cost_matrix[i][j] = int(compute_distance(facility_coords[i], customer_coords[j]))

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

def compute_subgradient(x_sol):
    return 1 - np.sum(x_sol, axis=0)

def step_length(gap, d, kappa=1.0):
    norm_sq = np.sum(d ** 2)
    return 0 if norm_sq == 0 else kappa * gap / norm_sq

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

def extract_clusters(x_sol, y_sol):
    clusters = []
    for i in range(m):
        if y_sol[i]:
            assigned = [j for j in range(n) if x_sol[i][j]]
            if assigned:
                clusters.append((i, assigned))
    return clusters

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
    return order

def get_distance(coords):
    N = len(coords)
    return [[compute_distance(coords[i], coords[j]) for j in range(N)] for i in range(N)]

# -------------------------------
# 실행부 및 시각화, .sol 출력
# -------------------------------
start_time = time.time()
result = lagrangian_heuristic()
if result is None:
    print("No feasible solution found.")
    exit()

x_final, y_final = result
clusters = extract_clusters(x_final, y_final)

plt.figure(figsize=(10, 10))
plt.scatter(depot_coord[0], depot_coord[1], color='red', s=300, marker='*', label='Depot', zorder=10)
colors = plt.cm.tab20(np.linspace(0, 1, len(clusters)))

sol_lines = []
total_cost = 0

for idx, (facility, customers) in enumerate(clusters, 1):
    cluster_coords = [depot_coord] + [customer_coords[j] for j in customers]
    route = nearest_neighbor_tsp(cluster_coords)
    dist_mat = get_distance(cluster_coords)
    cost, opt_route = two_opt(0, route[1:-1], dist_mat, cluster_coords)
    if opt_route[0] != 0:
        opt_route = [0] + opt_route
    if opt_route[-1] != 0:
        opt_route = opt_route + [0]
    xs = [cluster_coords[i][0] for i in opt_route]
    ys = [cluster_coords[i][1] for i in opt_route]
    plt.plot(xs, ys, 'o-', color=colors[idx-1], alpha=0.7, label=f"Route {idx}")
    plt.scatter([cluster_coords[i][0] for i in opt_route[1:-1]],
                [cluster_coords[i][1] for i in opt_route[1:-1]],
                color=colors[idx-1], s=50, alpha=0.9, label=None)
    # .sol 라인 생성 (고객 1-based, depot 제외)
    sol_customers = [customers[i-1]+1 for i in opt_route[1:-1]]  # 1-based
    sol_line = f"Route #{idx}: {' '.join(str(c) for c in sol_customers)}"
    sol_lines.append(sol_line)
    total_cost += cost

plt.title("All TSP Routes (2-opt + NN, SSCFL Partition)")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis('equal')
plt.legend(loc="best", fontsize='small')
plt.show()

# .sol 포맷 출력 및 파일 저장
for line in sol_lines:
    print(line)
print(f"Cost {int(total_cost)}")

with open("output.sol", "w") as f:
    for line in sol_lines:
        f.write(line + "\n")
    f.write(f"Cost {int(total_cost)}\n")

print(f"\n✅ 전체 실행 시간: {time.time() - start_time:.2f}초")
print("✅ .sol 형식의 결과는 output.sol로 저장되었습니다.")
