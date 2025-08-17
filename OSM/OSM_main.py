import time
import json
import matplotlib.pyplot as plt

from OSM_ACO import ACO_VRPB
from OSM_util import check_feasible


def OSM_run(problem_info, time_limit=60, log=True):

    start = time.time()
    K = problem_info['K']
    capa = problem_info['capa']
    node_types_list = problem_info['node_types']
    node_demands_list = problem_info['node_demands']
    node_coords = problem_info['node_coords']
    dist_mat = problem_info['dist_mat']
    
    demands = []
    for i, node_type_val in enumerate(node_types_list):
        demand_value = node_demands_list[i]
        if node_type_val == 1:  # Linehaul
            demands.append(demand_value)
        elif node_type_val == 2:  # Backhaul
            demands.append(-demand_value)
        else:  # Depot
            demands.append(0)

    aco_solver = ACO_VRPB(
        iterations=100,
        ants=K,
        alpha=1,
        beta=5,
        ro=0.8,
        th = 80,
        q0 = 0.9
    )

    final_solution = aco_solver.solve(
        K=K,
        capa=capa,
        node_coords=node_coords,
        dist_mat=dist_mat,
        demands=demands
    )
    
    if final_solution and final_solution[1] != float('inf'):
        solution_routes = final_solution[0] # Depot이 포함된 경로
        elapsed_time = time.time() - start

        if log:
            print("\n--- Final Solution Validation ---")
        final_cost = check_feasible(
            problem_info=problem_info,
            sol=solution_routes,
            elapsed=elapsed_time,
            timelimit=time_limit
        )

        if final_cost > 0:
            if log:
                print(f"Validation PASSED. Final valid cost: {final_cost:.2f}")
                print(solution_routes)
            return solution_routes
        else:
            if log:
                print("Validation FAILED. The solution is not feasible.")
                print(solution_routes)
            return [] # 유효하지 않으므로 빈 리스트 반환
            
    else:
        if log:
            print("ACO Solver could not find a valid solution.")
        return []
    


if __name__ == "__main__":
    file_path = 'C:/Users/risklab/Desktop/code/25SummerStudy/instances/problem_50_0.85.json'
    
    with open(file_path, 'r') as f:
        problem_info = json.load(f)

    OSM_run(problem_info=problem_info, time_limit=60)
    