import re
import matplotlib.pyplot as plt
import numpy as np
import vrplib
import os

instance_name = "A-n32-k5"
vrp_path = f"./vrp_instances/{instance_name}.vrp"
sol_path = f"./vrp_instances/{instance_name}.sol"

if not os.path.exists(vrp_path):
    raise FileNotFoundError(f"{vrp_path} 파일이 존재하지 않습니다.")
if not os.path.exists(sol_path):
    raise FileNotFoundError(f"{sol_path} 파일이 존재하지 않습니다.")

problem = vrplib.read_instance(vrp_path)
coords_dict = problem["node_coord"]  # {1: (x1, y1), 2: (x2, y2), ...}
depot = problem["depot"]

# depot robust 변환 (보통 1)
if isinstance(depot, (list, tuple, np.ndarray)):
    depot_idx = int(np.asarray(depot).item())
else:
    depot_idx = int(depot)
depot_coord = coords_dict[depot_idx]

# .sol 파일 파싱
def parse_sol_file(path):
    routes = []
    with open(path, "r") as f:
        for line in f:
            if line.startswith('Route'):
                nums = list(map(int, re.findall(r'\d+', line)))
                routes.append(nums[1:])  # Route #번호는 버림
    return routes

routes = parse_sol_file(sol_path)

# 시각화
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
    plt.title(f"Routes Visualization for {instance_name}", fontsize=15)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

visualize_solution(coords_dict, routes, depot_idx)
