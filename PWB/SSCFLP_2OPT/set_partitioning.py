import random
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB
import numpy as np

# 1. Data Generation
np.random.seed(42)

n = 30
m = 4
x = np.random.uniform(0, 100, n)
y = np.random.uniform(0, 100, n)
xy = np.vstack([x, y])  # shape: (2, 30)
u = np.random.uniform(0, 100, n)
v = np.random.uniform(0, 100, n)
uv = np.vstack([u, v])  # shape: (2, 30)

facility_capacity = 5000
depot_coord = (50, 50)
customer_coords = list(zip(xy[0], xy[1]))
facility_coords = list(zip(uv[0], uv[1]))
demands = [random.uniform(100, 600) for _ in range(n)]
capacities = [facility_capacity for _ in range(m)]
area_x, area_y = 100, 100

# n = 300
# m = 40
# area_x, area_y = 24000, 32000
# demand_mean, demand_std = 500, 200
#  facility_capacity = 5000
# depot_coord = (12000, 16000)
# customer_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(n)]
# facility_coords = [(random.uniform(0, area_x), random.uniform(0, area_y)) for _ in range(m)]
# demands = [max(1, int(random.gauss(demand_mean, demand_std))) for _ in range(n)]
# capacities = [facility_capacity for _ in range(m)]

# 2. Cost Matrix
def euclidean(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5
transport_cost = [
    [euclidean(customer_coords[i], facility_coords[j]) for j in range(m)]
    for i in range(n)
]

# 3. Gurobi Model
model = gp.Model("SSCFLP")
x = model.addVars(n, m, vtype=GRB.BINARY, name="x")
y = model.addVars(m, vtype=GRB.BINARY, name="y")

# Objective: Only transportation cost
model.setObjective(
    gp.quicksum(transport_cost[i][j]*x[i, j] for i in range(n) for j in range(m)),
    GRB.MINIMIZE
)

# Constraints
for i in range(n):
    model.addConstr(gp.quicksum(x[i, j] for j in range(m)) == 1)
for j in range(m):
    model.addConstr(gp.quicksum(demands[i]*x[i, j] for i in range(n)) <= capacities[j]*y[j])
for i in range(n):
    for j in range(m):
        model.addConstr(x[i, j] <= y[j])

model.Params.OutputFlag = 0
model.optimize()

# 4. Save solution in .sol format
def save_solution(model, x_vars, y_vars, filename="solution.sol"):
    with open(filename, "w") as f:
        for j in range(m):
            if y_vars[j].X > 0.5:
                f.write(f"facility {j} opened at {facility_coords[j]}\n")
        for i in range(n):
            for j in range(m):
                if x_vars[i, j].X > 0.5:
                    f.write(f"customer {i} assigned to facility {j}\n")

save_solution(model, x, y)

# 5. Visualization
fig, ax = plt.subplots(figsize=(6, 6))
fac_colors = ["blue" if y[j].X > 0.5 else "gray" for j in range(m)]
for j in range(m):
    ax.scatter(*facility_coords[j], marker='^', c=fac_colors[j], s=140, label="Facility" if j==0 else "")

for i in range(n):
    for j in range(m):
        if x[i, j].X > 0.5:
            ax.plot([customer_coords[i][0], facility_coords[j][0]],
                    [customer_coords[i][1], facility_coords[j][1]], 'g-', alpha=0.25, linewidth=0.8)
    ax.scatter(*customer_coords[i], marker='o', c="orange", s=20, label="Customer" if i==0 else "")

ax.scatter(*depot_coord, marker="*", c="red", s=240, label="Depot")
ax.set_xlim(0, area_x)
ax.set_ylim(0, area_y)
ax.set_title("SSCFLP Assignment Result (Fixed Cost = 0)")
handles, labels = ax.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
ax.legend(by_label.values(), by_label.keys(), loc="best")
plt.tight_layout()
plt.show()
