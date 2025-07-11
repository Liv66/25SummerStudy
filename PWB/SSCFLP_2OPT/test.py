import numpy as np
import matplotlib.pyplot as plt
import time
import random

# -------------------------------
# 인스턴스 파라미터 설정
# -------------------------------
n = 150  # 고객 수
m = 20  # 시설 수
area_x, area_y = 24000, 32000
demand_mean, demand_std = 500, 200
facility_capacity = 5000

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
# Greedy Knapsack per Facility
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
# Subgradient Functions
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
# Lagrangian Heuristic Main Loop
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
        if feasible:
            x_feas, y_feas = feasible
            upper = np.sum(cost_matrix * x_feas) + np.sum(np.array(fixed_costs) * y_feas)
            best_upper = min(best_upper, upper)

        d = compute_subgradient(x_sol)
        gap = best_upper - g_val
        t = step_length(gap, d)
        u = u + t * d

        print(f"Iter {it:2d} | Lower Bound: {g_val:.2f} | Upper Bound: {best_upper:.2f} | Gap: {gap:.2f}")
        if gap < 1:
            break

    return x_feas, y_feas

# -------------------------------
# 시각화 함수
# -------------------------------
def visualize_solution(x_sol, y_sol):
    plt.figure(figsize=(10, 8))
    for i in range(m):
        plt.scatter(*facility_coords[i], color='blue' if y_sol[i] else 'gray',
                    s=200 if y_sol[i] else 100, marker='s',
                    label='Facility' if i == 0 else "")
    for j in range(n):
        plt.scatter(*customer_coords[j], color='red', s=40, label='Customer' if j == 0 else "")
    for i in range(m):
        for j in range(n):
            if x_sol[i, j]:
                x = [facility_coords[i][0], customer_coords[j][0]]
                y = [facility_coords[i][1], customer_coords[j][1]]
                plt.plot(x, y, 'k-', alpha=0.3)
    plt.title("SSCFL Solution Visualization")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)
    plt.show()


# -------------------------------
# 실행부 + 시간 측정
# -------------------------------
start_time = time.time()
x_final, y_final = lagrangian_heuristic()
end_time = time.time()

visualize_solution(x_final, y_final)


print(f"\n✅ 최적화 실행 시간: {end_time - start_time:.4f}초")
