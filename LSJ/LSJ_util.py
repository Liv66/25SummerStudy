import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from collections import namedtuple

# --------------------------------------------------------------------------------
# 데이터 구조 (클래스 및 Named Tuple)
# --------------------------------------------------------------------------------
class VRPColumn:
    def __init__(self, path, cost, route_type="Unknown"):
        self.path = path; self.cost = cost; self.route_type = route_type; self.reduced_cost = float('inf'); self.ever_basic = False
    def __repr__(self):
        return f"Type: {self.route_type:<8s}, Path: {' -> '.join(map(str, self.path))}, Cost: {self.cost:.2f}, RC: {self.reduced_cost:.2f}"

Label = namedtuple('Label', ['cost', 'load', 'node', 'path'])

# --------------------------------------------------------------------------------
# 공용 헬퍼 함수
# --------------------------------------------------------------------------------
def get_route_type(path, nodes):
    if len(path) <= 2: return "Empty"
    types = {nodes[i]['type'] for i in path[1:-1]}
    if not types: return "Empty"
    if len(types) == 1: return types.pop()
    return 'mixed'

def calculate_path_cost(path, dist_matrix):
    return sum(dist_matrix[path[i]][path[i+1]] for i in range(len(path) - 1))

def is_valid_vrpb_route(path, nodes, capacity):
    if len(path) <= 2: return True
    is_backhaul_seen = False
    for node_idx in path[1:-1]:
        node_info = nodes[node_idx]
        if node_info['type'] == 'backhaul': is_backhaul_seen = True
        elif node_info['type'] == 'linehaul' and is_backhaul_seen: return False
    total_linehaul_demand = sum(nodes[n]['demand'] for n in path[1:-1] if nodes[n]['type'] == 'linehaul')
    total_backhaul_demand = sum(nodes[n]['demand'] for n in path[1:-1] if nodes[n]['type'] == 'backhaul')
    return total_linehaul_demand <= capacity and total_backhaul_demand <= capacity

def plot_solution(nodes, final_routes, title):
    plt.figure(figsize=(14, 10))
    depot = nodes[0]
    plt.scatter([c['x'] for c in nodes if c['type'] == 'linehaul'], [c['y'] for c in nodes if c['type'] == 'linehaul'], c='skyblue', label='Linehaul', s=60, zorder=3, edgecolors='black')
    plt.scatter([c['x'] for c in nodes if c['type'] == 'backhaul'], [c['y'] for c in nodes if c['type'] == 'backhaul'], c='salmon', label='Backhaul', s=60, zorder=3, edgecolors='black')
    plt.scatter(depot['x'], depot['y'], c='red', marker='s', s=150, label='Depot', zorder=3, edgecolors='black')
    for node in nodes[1:]: plt.text(node['x'], node['y'] + 500, str(node['id']), fontsize=9, ha='center')
    colors = list(mcolors.TABLEAU_COLORS.values())
    for i, route in enumerate(final_routes):
        color = colors[i % len(colors)]
        for j in range(len(route) - 1):
            start_node, end_node = nodes[route[j]], nodes[route[j+1]]
            plt.annotate(text="", xy=(end_node['x'], end_node['y']), xytext=(start_node['x'], start_node['y']), arrowprops=dict(arrowstyle="->", connectionstyle="arc3", color=color, lw=1.5), size=15, zorder=2)
    plt.title(title, fontsize=16); plt.xlabel("X-coordinate"); plt.ylabel("Y-coordinate")
    handles, labels = plt.gca().get_legend_handles_labels(); plt.legend(list(dict(zip(labels, handles)).values()), list(dict(zip(labels, handles)).keys())); plt.grid(True); plt.show()