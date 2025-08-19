import json
import time
from cwj_algorithm import VRPB_CG_Heuristic

def cwj_main(problem_info):
    K = problem_info['K']
    node_type = problem_info['node_types']
    node_demand = problem_info['node_demands']
    capa = problem_info['capa']
    dist_mat = problem_info['dist_mat']

    print("========== [DEBUG] Problem Info ==========")
    print(f"Number of vehicles (K): {K}")
    print(f"Vehicle capacity (Q): {capa}")
    print(f"Total number of nodes (N): {len(node_type)}")
    print(f"Number of linehaul customers: {node_type.count(1)}")
    print(f"Number of backhaul customers: {node_type.count(2)}")
    print(f"Depot index (always 0): demand={node_demand[0]}, type={node_type[0]}")
    print(f"Distance matrix size: {len(dist_mat)} x {len(dist_mat[0])}")
    print("==========================================")

    solution = VRPB_CG_Heuristic(problem_info)
    print("Final solution:")
    total_cost = 0
    for route, cost in solution:
        print(f"Route: {route} | Cost: {int(cost)}")
        total_cost += cost
    print(f"Total Cost: {int(total_cost)}")

if __name__ == '__main__':
    with open('/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/problem_100_0.7.json', 'r') as f:
        instance = json.load(f)

    start_time = time.time()
    cwj_main(instance)
    end_time = time.time()
    elapsed = end_time - start_time
    print(f"\n실행 시간: {elapsed:.2f}초")