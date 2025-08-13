import json
import random
import time

from util_v1 import *
from construction import *
from method import *
from solver import *
from Solution import *


def instance_generator(problem, N=50, capa=3000, line_p=0.7):
    problem_info = {}
    nodes_coord = [(12000, 16000)] + [(random.uniform(0, 24000), random.uniform(0, 32000)) for _ in range(N - 1)]
    demands = [0]
    for _ in range(N - 1):
        demand = 0
        while demand <= 0:
            demand = int(random.gauss(500, 200))
        demands.append(demand)

    num_line = int(N * line_p)
    node_type = [2 for _ in range(N)]
    node_type[0] = 0
    line_list = random.sample(range(1, N), num_line)
    for i in line_list:
        node_type[i] = 1

    line_K = bin_packing([demands[i] for i in range(N) if node_type[i] == 1], capa)
    back_K = bin_packing([demands[i] for i in range(N) if node_type[i] == 2], capa)
    K = max(line_K, back_K)
    dist_mat = get_distance(nodes_coord)

    problem_info['N'] = N
    problem_info['K'] = K
    problem_info['capa'] = capa
    problem_info['node_demands'] = demands
    problem_info['node_types'] = node_type
    problem_info['node_coords'] = nodes_coord
    problem_info['dist_mat'] = dist_mat

    with open(problem, "w", encoding='utf-8') as f:
        json.dump(problem_info, f, ensure_ascii=False, indent=4)


def main():
    N = 50
    line_p = 0.7
    capa = 2000
    problem = f"instances/problem_{N}_{line_p}.json"
    try:
        with open(problem, "r", encoding='utf-8') as f:
            problem_info = json.load(f)

    except FileNotFoundError:
        instance_generator(problem, N=N, line_p=line_p, capa=capa)
        with open(problem, "r", encoding='utf-8') as f:
            problem_info = json.load(f)
            
    start = time.time()

    # ILS를 위한 구성 요소들
    construct = GreedyConstructionStrategy()
    local_search = FirstImprovementStrategy()

    # 메인 Solver를 ILS로 변경
    solver = IteratedLocalSearchSolver(construct, local_search)
    start_time = time.time()
    time_limit = 60
    solution = solver.solve(problem_info, start_time, time_limit) # CVRPBSolution object

    elapsed = time.time() - start_time
    time_limit = 60
    
    final_routes = [list(route) for route in solution.get_routes()]

    print("--- Final Solution ---")
    print(solution)
    distance_cost = sum(r.get_distance_cost() for r in solution.get_routes())
    print(f"Final Distance Cost: {distance_cost:.2f}")
    
    print("\n--- Feasibility Check ---")
    obj = check_feasible(problem_info, final_routes, elapsed, time_limit)
    if obj > 0:
        print(f"Feasibility Check PASSED. Objective value: {obj:.2f}")
    else:
        print("Feasibility Check FAILED!")
        
    plot_vrpb(problem_info, final_routes, f'Optimized VRPB Solution\nObj: {obj:.2f}')

if __name__ == '__main__':
    main()
