# cwj_main.py
import json
from cwj_master_problem import solve_master_problem
from cwj_initial_patterns import generate_initial_patterns
from cwj_dual_route_generator import generate_dual_routes
from cwj_local_search import improve_solution
from cwj_master_postprocess import generate_augmented_routes
from cwj_phase2_postprocess import prune_redundant_customers


def VRPB_CG_Heuristic(problem_info):
    K = problem_info['K']
    Q = problem_info['capa']
    node_types = problem_info['node_types']
    node_demands = problem_info['node_demands']
    dist_mat = problem_info['dist_mat']

    # Phase 1: 초기 feasible 해 및 route pool 구성
    route_pool = generate_initial_patterns(K, Q, node_types, node_demands, dist_mat)
    best_solution, duals, missed_customers = solve_master_problem(route_pool, node_types, dist_mat, K, relax=False)
    best_solution = improve_solution(best_solution, route_pool, node_types, node_demands, dist_mat, Q)

    # Phase 1.5: Master infeasible 시 fallback route 생성
    if missed_customers:
        print(f"[POSTPROCESS] Fallback route 생성 시작... missed={missed_customers}")
        augmented_routes = generate_augmented_routes(route_pool, missed_customers, Q, node_types, node_demands, dist_mat)
        route_pool.extend(augmented_routes)
        
        # 다시 master problem 풀기
        best_solution, duals, missed_customers = solve_master_problem(route_pool, node_types, dist_mat, K, relax=True)
        best_solution = improve_solution(best_solution, route_pool, node_types, node_demands, dist_mat, Q)

    print("Phase 1 Solution")
    total_cost = 0
    for idx, (route, cost) in enumerate(best_solution):
        print(f"Route {idx+1}: {route} | Cost: {int(cost)}")
        total_cost += cost
    print(f"Total Cost before pruning: {int(total_cost)}\n")

    # Phase 2: reduced cost route 생성 및 개선 반복
    for _ in range(1):  # 최대 10회 반복
        new_routes = generate_dual_routes(duals, route_pool, node_types, node_demands, dist_mat, Q)
        if not new_routes:
            break
        route_pool.extend(new_routes)
        print(f"[DEBUG] Initial route pool size: {len(route_pool)}")
        best_solution, duals, missed_customers = solve_master_problem(route_pool, node_types, dist_mat, K, relax=True)
        best_solution = improve_solution(best_solution, route_pool, node_types, node_demands, dist_mat, Q)
    
    print("Phase 2 Solution")
    total_cost = 0
    for idx, (route, cost) in enumerate(best_solution):
        print(f"Route {idx+1}: {route} | Cost: {int(cost)}")
        total_cost += cost
    print(f"Total Cost before pruning: {int(total_cost)}\n")
    
    # Phase 3: 중복 고객 제거를 통한 partitioning 근사
    print("[PHASE 3] Pruning duplicate customers from solution...")
    best_solution = prune_redundant_customers(best_solution, node_types, node_demands, dist_mat, Q)

    return best_solution

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
    with open('/Users/michael/Desktop/RiskLAB./Study/25SummerStudy/instances/problem_20_0.7.json', 'r') as f:
        instance = json.load(f)
    cwj_main(instance)